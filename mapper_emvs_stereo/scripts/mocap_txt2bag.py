import numpy as np
import os
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
from tqdm import tqdm
import argparse

parser = argparse.ArgumentParser()
# parser.add_argument("bag", help="ROS bag file to extract")
parser.add_argument("--path_prefix", help="Folder that contains input pose")
# parser.add_argument("--pose_topic", default="/pose", help="Pose topic")
args = parser.parse_args()
path_prefix = args.path_prefix
# path_prefix = '/mnt/HD4TB/data/tum-vie/slide-vi_gt_data/'

pose_fname = path_prefix + 'mocap_data.txt'
pos = np.loadtxt(pose_fname, delimiter=' ', skiprows=1)

bag_file_name = path_prefix + 'pose'
# topic = args.pose_topic
topic = '/pose'

with rosbag.Bag('{}.bag'.format(bag_file_name), 'w') as bag:
    for i in tqdm(range(pos.shape[0])):
        msg = PoseStamped()
        msg.pose.position.x = pos[i, 1]
        msg.pose.position.y = pos[i, 2]
        msg.pose.position.z = pos[i, 3]
        msg.pose.orientation.x = pos[i, 4]
        msg.pose.orientation.y = pos[i, 5]
        msg.pose.orientation.z = pos[i, 6]
        msg.pose.orientation.w = pos[i, 7]
        msg.header.stamp = rospy.Time.from_sec(pos[i, 0] / 1e6)
        msg.header.frame_id = 'map'
        bag.write(topic, msg, msg.header.stamp)
