import numpy as np
import matplotlib.pyplot as plt
import os, sys
import cv2
from tqdm import tqdm
from precision_completeness import precision_completeness
from depth_metrics import error_metrics
import yaml


def load_intrinsics_yaml(prefix):
    with open(os.path.join(prefix, 'cam_to_cam.yaml'), "r") as stream:
        try:
            cam_cam = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    with open(os.path.join(prefix, 'cam_to_lidar.yaml'), "r") as stream:
        try:
            cam_lidar = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    Q = np.array(cam_cam['disparity_to_depth']['cams_03'])
    R_rect0 = np.array(cam_cam['extrinsics']['R_rect0'])
    T_rect0_0 = np.eye(4)
    T_rect0_0[:3, :3] = R_rect0
    int_cam_0 = np.array(cam_cam['intrinsics']['cam0']['camera_matrix'])
    K_cam_0 = np.array([int_cam_0[0], 0, int_cam_0[2],
                        0, int_cam_0[1], int_cam_0[3],
                        0, 0, 1]).reshape((3, 3))
    dist_cam_0 = np.array(cam_cam['intrinsics']['cam0']['distortion_coeffs'])
    int_cam_3 = np.array(cam_cam['intrinsics']['cam3']['camera_matrix'])
    K_cam_3 = np.array([int_cam_3[0], 0, int_cam_3[2],
                        0, int_cam_3[1], int_cam_3[3],
                        0, 0, 1]).reshape((3, 3))
    dist_cam_3 = np.array(cam_cam['intrinsics']['cam3']['distortion_coeffs'])
    return Q, T_rect0_0, K_cam_0, dist_cam_0, K_cam_3, dist_cam_3


h = 480
w = 640
path = '/home/suman/data/rpg/DSEC/zurich_city_04_a/zurich_city_04_a_disparity_event/'
flist = os.listdir(path)
Q, T_rect0_0, K_cam_0, dist_cam_0, K_cam_3, dist_cam_3 = load_intrinsics_yaml(path + '../zurich_city_04_a_calibration/')
K_0 = np.zeros((3, 4))
K_0[:, :3], _ = cv2.getOptimalNewCameraMatrix(K_cam_0, dist_cam_0, (640, 480), alpha=0)
b = 0.6
f = K_0[0, 0]
dpoints_name = 'depth_points_fused_2.txt'
event_start_time = 36470.599680000
min_depth = 4
max_depth = 50
start = 0
stop = 35
d_path = '/home/suman/data/semvs_results/DSEC/zurich_04_a_full/test2/'
d_path_mono = d_path
f_ts = np.loadtxt('/home/suman/data/rpg/DSEC/zurich_city_04_a/zurich_city_04_a_disparity_timestamps.txt')
f_path = '/home/suman/data/rpg/DSEC/zurich_city_04_a/zurich_city_04_a_disparity_event/'

gt_depth_consolidated = []
est_depth_consolidated = []
est_depth_consolidated_mono = []

thicken_edges = False
# thicken_edges = True


def get_mcemvs_depth(path, time_prefix, suffix):
    fname = path + time_prefix + suffix
    depthmap = np.zeros((h, w)) + 255
    depth_points = np.loadtxt(fname).reshape(-1, 3)
    if depth_points.size != 0:
        depthmap[depth_points[:, 1].astype(int), depth_points[:, 0].astype(int)] = depth_points[:, 2]
    if thicken_edges:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        depthmap = cv2.morphologyEx(depthmap, cv2.MORPH_ERODE, kernel)
    depthmap = np.ma.array(depthmap, mask=(depthmap == 255))
    return depthmap


# times = [8, 11.4, 12.5, 14.2, 31.2]
invdname = 'inv_depth_colored_dilated_fused_2.png'
dname = 'depth_map_fused_2.png'
files = os.listdir(d_path)
times = []

for file in tqdm(sorted(files)):
    if file.endswith(invdname):
        prefix = file[:file.find(invdname)]
        t = float(prefix)
        times.append(t)
        if t<start or t>stop:
            continue

        depth_meter = get_mcemvs_depth(d_path, prefix, dpoints_name)

        frame_id = (np.abs(f_ts.astype(np.float64) - (t + event_start_time) * 1e6)).argmin()
        disp = plt.imread(f_path + str(frame_id * 2).zfill(6) + '.png')
        time_gt = f_ts[frame_id]
        dt = abs(float(time_gt) * 1e-6 - event_start_time - t)
        # print('Time difference between mcemvs and gt: ' + str(dt))
        if dt >= 0.1:
            print('GT depth is too far away in time.... Skipping ...........')
            continue

        # convert disparity to depth and project to undistorted left event camera frame
        d = disp.astype(np.float32) * 256
        _3dImage = cv2.reprojectImageTo3D(d, Q)
        points = (_3dImage[np.where(_3dImage[:, :, 2] < np.inf)]).T
        P_homo = np.r_[points, np.ones((1, points.shape[1]))]
        P_new = np.linalg.inv(T_rect0_0) @ P_homo
        px = K_0 @ P_new
        px[0, :] /= px[2, :]
        px[1, :] /= px[2, :]
        out_d = np.zeros_like(d)
        try:
            out_d[px[1, :].astype(int), px[0, :].astype(int)] = P_new[2, :]
        except:
            print('Depth out of bounds')
            pass
        masked_gt_depth = np.ma.array(out_d, mask=(out_d < 0.05))

        gt_depth_consolidated.append(masked_gt_depth)
        est_depth_consolidated.append(depth_meter)

gt_depth_consolidated = np.ma.array(gt_depth_consolidated)
est_depth_consolidated = np.ma.array(est_depth_consolidated)
est_depth_consolidated_mono = np.ma.array(est_depth_consolidated_mono)

scene_depth = np.max(gt_depth_consolidated[~ np.isnan(gt_depth_consolidated)])

print('----------------Stereo EMVS---------------')
error = np.absolute(gt_depth_consolidated - est_depth_consolidated)
# error = np.ma.array(error, mask=(error >= 10))
print("Mean " + str(np.ma.mean(error)))
print("Median " + str(np.ma.median(error)))
print('Max depth: ' + str(scene_depth))
print('Number of error points: ' + str(np.ma.count(error)))
error_metrics(est_depth_consolidated, gt_depth_consolidated, name='Alg 1', b=b, f=f)
ax1, ax2, ax3 = precision_completeness(est_depth_consolidated, gt_depth_consolidated, 'Alg 1')
