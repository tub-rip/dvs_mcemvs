/*
* \file main.cpp
* \brief main source
* \author (1) Suman Ghosh
* \date 2022-09-01
* \author (2) Guillermo Gallego
* \date 2022-09-01
* Copyright/Rights of Use:
* 2022, Technische Universität Berlin
* Prof. Guillermo Gallego
* Robotic Interactive Perception
* Marchstrasse 23, Sekr. MAR 5-5
* 10587 Berlin, Germany
*/

#include <mapper_emvs_stereo/data_loading.hpp>
#include <mapper_emvs_stereo/mapper_emvs_stereo.hpp>
#include <mapper_emvs_stereo/utils.hpp>
#include <mapper_emvs_stereo/calib.hpp>
#include <mapper_emvs_stereo/process1.hpp>
#include <mapper_emvs_stereo/process2.hpp>
#include <mapper_emvs_stereo/process5.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui.hpp>
#include <pcl/io/pcd_io.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>
#include <fstream>

// Input parameters

// I/O paths
DEFINE_string(bag_filename, "", "Path to the rosbag");
DEFINE_string(bag_filename_left, "input.bag", "Path to the rosbag");
DEFINE_string(bag_filename_right, "input.bag", "Path to the rosbag");
DEFINE_string(bag_filename_pose, "input.bag", "Path to the rosbag");
DEFINE_string(out_path, "./", "Output path prefix");

// Calibration
DEFINE_string(calib_type, "yaml", "Choose how to parse calib file, varies according to dataset.");
DEFINE_string(calib_path, "stereo_pinhole.yaml", "Output path prefix");
DEFINE_string(mocap_calib_path, "stereo_pinhole.yaml", "Output path prefix");

// ROS topics
DEFINE_string(event_topic0, "/davis_left/events", "Name of the event topic (default: /dvs/left/events)");
DEFINE_string(event_topic1, "/davis_right/events", "Name of the event topic (default: /dvs/right/events)");
DEFINE_string(event_topic2, "", "Name of the event topic (default: emtpy string)");
DEFINE_string(camera_info_topic0, "/davis_left/camera_info", "Name of the camera info topic (default: /dvs/left/camera_info)");
DEFINE_string(camera_info_topic1, "/davis_right/camera_info", "Name of the camera info topic (default: /dvs/right/camera_info)");
DEFINE_string(camera_info_topic2, "", "Name of the camera info topic (default: empty string)");
DEFINE_string(pose_topic, "/optitrack/davis_stereo", "Name of the pose topic (default: /optitrack/dvs)");

DEFINE_double(offset0, 0, "Event msg timestamp offset wrt poses");
DEFINE_double(offset1, 0, "Event msg timestamp offset wrt poses");
DEFINE_double(offset2, 0, "Event msg timestamp offset wrt poses");

DEFINE_double(start_time_s, 0.0, "Start time in seconds (default: 0.0)");
DEFINE_double(stop_time_s, 1000.0, "Stop time in seconds (default: 1000.0)");

// Disparity Space Image (DSI) parameters
DEFINE_int32(dimX, 0, "X dimension of the voxel grid (if 0, will use the X dim of the event camera) (default: 0)");
DEFINE_int32(dimY, 0, "Y dimension of the voxel grid (if 0, will use the Y dim of the event camera) (default: 0)");
DEFINE_int32(dimZ, 100, "Z dimension of the voxel grid (default: 100) must be <= 256");
DEFINE_double(fov_deg, 0.0, "Field of view of the DSI, in degrees (if < 10, will use the FoV of the event camera) (default: 0.0)");
DEFINE_double(min_depth, 0.3, "Min depth, in meters (default: 0.3)");
DEFINE_double(max_depth, 5.0, "Max depth, in meters (default: 5.0)");

// Depth map parameters (selection and noise removal)
DEFINE_int32(adaptive_threshold_kernel_size, 5, "Size of the Gaussian kernel used for adaptive thresholding. (default: 5)");
DEFINE_double(adaptive_threshold_c, 5., "A value in [0, 255]. The smaller the noisier and more dense reconstruction (default: 5.)");
DEFINE_int32(median_filter_size, 5, "Size of the median filter used to clean the depth map. (default: 5)");
DEFINE_bool(save_mono, false, "If set to true, results for monocular EMVS (left and right camera) will be saved as well");

// Point cloud parameters (noise removal). Section 5.2.4 in the IJCV paper.
DEFINE_double(radius_search, 0.05, "Size of the radius filter. (default: 0.05)");
DEFINE_int32(min_num_neighbors, 3, "Minimum number of points for the radius filter. (default: 3)");
DEFINE_bool(late_fusion, false, "Enable optional late fusion for comparison against proposed early fusion at DSI level");

DEFINE_int32(process_method, 1, "Select processing method");
DEFINE_int32(num_intervals, 4, "Number of sub-intervals to divide into for process 2 (temporal fusion)");
DEFINE_double(ts, (FLAGS_start_time_s + FLAGS_stop_time_s)/2, "Explicitly set DSI reference at this timestamp; default: middle of trajectory");
DEFINE_double(rv_pos, 0, "Set RV position shift along the baseline from the left camera center (implemented only for process 1); default: left camera center");
DEFINE_bool(forward_looking, false, "Put RV at end of trajectory if this flag is true");
DEFINE_int32(stereo_fusion, 2, "Fusion function for stereo DSI.. default: HM");
DEFINE_int32(temporal_fusion, 4, "DSI fusion function across time.. default: AM");

// Parameters for full sequence processing
DEFINE_bool(full_seq, false, "If set true, the whole sequence is divided into chunks and are processed independently");
DEFINE_bool(save_conf_stats, false, "If set true, saves txt file containing min (non-zero) and max confidence values for each DSI");
DEFINE_double(duration, 3, "Duration of events used in a single DSI, default: 3s.");
DEFINE_double(out_skip, 10, "Time period of mapping; default: 10s.");
DEFINE_double(max_confidence, 0, "Manually set this number as the upper limit of the DSI range before normalization to [0, 255]; If set to 0, the max value of the DSI becomes its upper limit.");

/*
 * Load a set of events and poses from a rosbag,
 * compute the disparity space image (DSI),
 * extract a depth map (and point cloud) from the DSI,
 * and save to disk.
 */
int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    sensor_msgs::CameraInfo camera_info_msg0, camera_info_msg1, camera_info_msg2;  // left and right
    // Create camera objects
    image_geometry::PinholeCameraModel cam0, cam1, cam2;

    // Load calibration from file (not from ROS bag)
    Eigen::Matrix4d mat4_1_0, mat4_2_0, mat4_hand_eye;
    if (FLAGS_calib_type == "eccv18")
        get_camera_calib_ECCV18(cam0, cam1, mat4_1_0, mat4_hand_eye);
    else if(FLAGS_calib_type == "esim")
        get_camera_calib_ESIM(cam0, cam1, mat4_1_0, mat4_hand_eye);
    else if(FLAGS_calib_type == "dvsgen3")
        get_camera_calib_DVS_Gen3(cam0, cam1, mat4_1_0, mat4_hand_eye);
    else if(FLAGS_calib_type == "yaml")
        get_camera_calib_yaml(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path);
    else if(FLAGS_calib_type == "yaml_mvsec")
        get_camera_calib_yaml_mvsec(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path);
    else if(FLAGS_calib_type == "slider")
        get_camera_calib_slider(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path);
    else if(FLAGS_calib_type == "hkust")
        get_camera_calib_hkust(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path);
    else if(FLAGS_calib_type == "evimo2")
        get_camera_calib_evimo2(cam0, cam1, cam2, mat4_1_0, mat4_2_0, mat4_hand_eye, FLAGS_calib_path);
    else if(FLAGS_calib_type == "json")
        get_camera_calib_json(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path, FLAGS_mocap_calib_path);
    else if(FLAGS_calib_type == "dsec_yaml")
        get_camera_calib_dsec_yaml(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path, FLAGS_mocap_calib_path);


    if (FLAGS_event_topic2 == "")
        cam2 = cam1;

    std::vector<dvs_msgs::Event> events0, events1, events2;
    std::map<ros::Time, geometry_utils::Transformation> poses;
    if (FLAGS_bag_filename!=""){
        FLAGS_bag_filename_left = FLAGS_bag_filename;
        FLAGS_bag_filename_right = FLAGS_bag_filename;
        FLAGS_bag_filename_pose = FLAGS_bag_filename;
    }

    // Initialize the DSI
    CHECK_LE(FLAGS_dimZ, 256) << "Number of depth planes should be <= 256";
    EMVS::ShapeDSI dsi_shape(FLAGS_dimX, FLAGS_dimY, FLAGS_dimZ,
                             FLAGS_min_depth, FLAGS_max_depth,
                             FLAGS_fov_deg);

    // Parameters to extract semi-dense depth map from DSI
    EMVS::OptionsDepthMap opts_depth_map;
    opts_depth_map.max_confidence = FLAGS_max_confidence;
    opts_depth_map.adaptive_threshold_kernel_size_ = FLAGS_adaptive_threshold_kernel_size;
    opts_depth_map.adaptive_threshold_c_ = FLAGS_adaptive_threshold_c;
    opts_depth_map.median_filter_size_ = FLAGS_median_filter_size;
    opts_depth_map.full_sequence = FLAGS_full_seq;
    opts_depth_map.save_conf_stats = FLAGS_save_conf_stats;
    opts_depth_map.save_mono = FLAGS_save_mono;
    opts_depth_map.rv_pos = FLAGS_rv_pos;

    if (FLAGS_full_seq)
    {
        //      int id0 = 0, id1 = 0, count = 0;

        for (double interval_start=FLAGS_start_time_s; interval_start+FLAGS_duration<=FLAGS_stop_time_s; interval_start+=FLAGS_out_skip)
        {

            std::vector<dvs_msgs::Event> interval_events0, interval_events1, interval_events2;
            std::map<ros::Time, geometry_utils::Transformation> interval_poses;

            double interval_stop = interval_start + FLAGS_duration;
            double ts;
            if(FLAGS_forward_looking)
                ts = interval_stop;
            else
                ts = (interval_start + interval_stop)/2;

            LOG(INFO) << "------------------ Time interval: " << interval_start << "-" << interval_stop << " s -------------------------";
            LOG(INFO)<<"Loading left events ...";
            data_loading::parse_rosbag(FLAGS_bag_filename_left, interval_events0, camera_info_msg0,
                                       FLAGS_event_topic0, FLAGS_camera_info_topic0, interval_start, interval_stop, FLAGS_offset0);
            LOG(INFO)<<"Loading right events ...";
            data_loading::parse_rosbag(FLAGS_bag_filename_right, interval_events1, camera_info_msg1,
                                       FLAGS_event_topic1, FLAGS_camera_info_topic1, interval_start, interval_stop, FLAGS_offset1);
            if(FLAGS_event_topic2 != "")
                data_loading::parse_rosbag(FLAGS_bag_filename, interval_events2, camera_info_msg2,
                                           FLAGS_event_topic2, FLAGS_camera_info_topic2, FLAGS_start_time_s, FLAGS_stop_time_s, FLAGS_offset2);
            LOG(INFO)<<"Loading poses ...";
            data_loading::parse_rosbag_gt(FLAGS_bag_filename_pose, interval_poses, FLAGS_pose_topic, FLAGS_start_time_s, FLAGS_stop_time_s);

            // Use linear interpolation to compute the camera pose for each event
            LinearTrajectory trajectory0 = LinearTrajectory(interval_poses);
            // Convert optitrack poses to left camera poses using hand-eye calibration
            geometry_utils::Transformation T_hand_eye(mat4_hand_eye);
            trajectory0.applyTransformationRight(T_hand_eye);
            // Trajectory of the second camera (right camera)
            LinearTrajectory trajectory1 = LinearTrajectory(interval_poses);
            trajectory1.applyTransformationRight(T_hand_eye); // so far, this is trajectory0
            geometry_utils::Transformation T_extr(mat4_1_0);
            trajectory1.applyTransformationRight( T_extr.inverse()); // now, it becomes trajectory1

            LinearTrajectory trajectory2;
            if (FLAGS_event_topic2 != ""){
                trajectory2 = LinearTrajectory(interval_poses);
                trajectory2.applyTransformationRight(T_hand_eye); // so far, this is trajectory0
                geometry_utils::Transformation T_extr(mat4_2_0);
                trajectory2.applyTransformationRight( T_extr.inverse());
            }

            //          interval_events0.clear();
            //          interval_events1.clear();
            //          while(true){
            //              id0++;
            //              if (events0[id0].ts.toSec() <= interval_start) {
            //                  continue;
            //                }
            //              if (events0[id0].ts.toSec() >= interval_stop){
            //                  break;
            //                }
            //              interval_events0.push_back(events0[id0]);
            //            }
            //          while(true){
            //              id1++;
            //              if (events1[id1].ts.toSec() <= interval_start) {
            //                  continue;
            //                }
            //              if (events1[id1].ts.toSec() >= interval_stop){
            //                  break;
            //                }
            //              interval_events1.push_back(events1[id1]);
            //            }

            {
                cv::Size full_resolution = cam0.fullResolution();
                cv::Mat event_image0 = cv::Mat(full_resolution,CV_8UC1);
                cv::Mat event_image1 = cv::Mat(full_resolution,CV_8UC1);
                accumulateEvents(interval_events0,true,event_image0);
                accumulateEvents(interval_events1,true,event_image1);
                cv::imwrite(FLAGS_out_path + std::to_string(ts) + "events_0.png",event_image0);
                cv::imwrite(FLAGS_out_path + std::to_string(ts) + "events_1.png",event_image1);
                if (FLAGS_event_topic2 != ""){
                    cv::Mat event_image2 = cv::Mat(full_resolution,CV_8UC1);
                    accumulateEvents(events2,true,event_image2);
                    cv::imwrite(FLAGS_out_path + "events_3.png",event_image2);
                }
            }

            cv::Mat depth_map, confidence_map, semidense_mask, depth_map_dense;
            LOG(INFO)<<"Initializing mapper fused";
            EMVS::MapperEMVS mapper_fused(cam0, dsi_shape), mapper_fused_camera_time(cam0, dsi_shape);
            mapper_fused.name = "fused";

            LOG(INFO)<<"Initializing mapper 0";
            EMVS::MapperEMVS mapper0(cam0, dsi_shape);
            mapper0.name="0";
            LOG(INFO)<<"Initializing mapper 1";
            EMVS::MapperEMVS mapper1(cam1, dsi_shape);
            mapper1.name="1";
            LOG(INFO)<<"Initializing mapper 2";
            EMVS::MapperEMVS mapper2(cam2, dsi_shape);
            mapper2.name="2";

            switch( FLAGS_process_method )
            {
            case 1:
            {
                // 1-3. Compute two DSIs (one for each camera) and fuse them
                if (events2.size()>0)
                    process_1(cam0,cam1,cam2, trajectory0,trajectory1,trajectory2, interval_events0, interval_events1, interval_events2, opts_depth_map,dsi_shape, mapper_fused, mapper0, mapper1, mapper2, std::string(FLAGS_out_path), ts, FLAGS_stereo_fusion);
                else
                    process_1(cam0,cam1,cam1, trajectory0,trajectory1,trajectory2, interval_events0, interval_events1, interval_events2, opts_depth_map,dsi_shape, mapper_fused, mapper0, mapper1, mapper2, std::string(FLAGS_out_path), ts, FLAGS_stereo_fusion);
                break;
            }
            case 2:
            {
                // 1-3. Split events into sub-intervals, compute fused DSIs per sub-interval, and fuse them into a single DSI
                mapper_fused.name = "time_camera";
                mapper_fused_camera_time.name = "camera_time";
                process_2(cam0,cam1,trajectory0,trajectory1, interval_events0, interval_events1, opts_depth_map,dsi_shape,FLAGS_num_intervals, mapper_fused, mapper_fused_camera_time, FLAGS_out_path, ts, FLAGS_stereo_fusion, FLAGS_temporal_fusion);
                break;
            }
            case 5:
            {
                // Shuffle event sub-intervals before fusing
                process_5(cam0,cam1,trajectory0,trajectory1, interval_events0, interval_events1, opts_depth_map, dsi_shape, FLAGS_num_intervals, mapper_fused, FLAGS_out_path, ts, FLAGS_stereo_fusion, FLAGS_temporal_fusion);
                break;
            }
            }
        }
    }
    else {

        LOG(INFO)<<"Loading poses ...";
        data_loading::parse_rosbag_gt(FLAGS_bag_filename_pose, poses, FLAGS_pose_topic, FLAGS_start_time_s, FLAGS_stop_time_s);
        LOG(INFO)<<"Loading left events ...";
        data_loading::parse_rosbag(FLAGS_bag_filename_left, events0, camera_info_msg0,
                                   FLAGS_event_topic0, FLAGS_camera_info_topic0, FLAGS_start_time_s, FLAGS_stop_time_s, FLAGS_offset0);
        LOG(INFO)<<"Loading right events ...";
        data_loading::parse_rosbag(FLAGS_bag_filename_right, events1, camera_info_msg1,
                                   FLAGS_event_topic1, FLAGS_camera_info_topic1, FLAGS_start_time_s, FLAGS_stop_time_s, FLAGS_offset1);
        if(FLAGS_event_topic2 != "")
            data_loading::parse_rosbag(FLAGS_bag_filename, events2, camera_info_msg2,
                                       FLAGS_event_topic2, FLAGS_camera_info_topic2, FLAGS_start_time_s, FLAGS_stop_time_s, FLAGS_offset2);

        // Use linear interpolation to compute the camera pose for each event
        LinearTrajectory trajectory0 = LinearTrajectory(poses);
        // Convert optitrack poses to left camera poses using hand-eye calibration
        geometry_utils::Transformation T_hand_eye(mat4_hand_eye);
        trajectory0.applyTransformationRight(T_hand_eye);
        // Trajectory of the second camera (right camera)
        LinearTrajectory trajectory1 = LinearTrajectory(poses);
        trajectory1.applyTransformationRight(T_hand_eye); // so far, this is trajectory0
        geometry_utils::Transformation T_extr(mat4_1_0);
        trajectory1.applyTransformationRight( T_extr.inverse()); // now, it becomes trajectory1

        LinearTrajectory trajectory2;
        if (FLAGS_event_topic2 != ""){
            trajectory2 = LinearTrajectory(poses);
            trajectory2.applyTransformationRight(T_hand_eye); // so far, this is trajectory0
            geometry_utils::Transformation T_extr(mat4_2_0);
            trajectory2.applyTransformationRight( T_extr.inverse());
        }

        {
            cv::Size full_resolution = cam0.fullResolution();
            cv::Mat event_image0 = cv::Mat(full_resolution,CV_8UC1);
            cv::Mat event_image1 = cv::Mat(full_resolution,CV_8UC1);
            accumulateEvents(events0,true,event_image0);
            accumulateEvents(events1,true,event_image1);
            cv::imwrite(FLAGS_out_path + "events_0.png",event_image0);
            cv::imwrite(FLAGS_out_path + "events_1.png",event_image1);
            if (FLAGS_event_topic2 != ""){
                cv::Mat event_image2 = cv::Mat(full_resolution,CV_8UC1);
                accumulateEvents(events2,true,event_image2);
                cv::imwrite(FLAGS_out_path + "events_2.png",event_image2);
            }
        }

        cv::Mat depth_map, confidence_map, semidense_mask;

        LOG(INFO)<<"Initializing mapper fused";
        EMVS::MapperEMVS mapper_fused(cam0, dsi_shape), mapper_fused_camera_time(cam0, dsi_shape);
        mapper_fused.name = "fused";
        LOG(INFO)<<"Initializing mapper 0";
        EMVS::MapperEMVS mapper0(cam0, dsi_shape);
        mapper0.name="0";
        LOG(INFO)<<"Initializing mapper 1";
        EMVS::MapperEMVS mapper1(cam1, dsi_shape);
        mapper1.name="1";
        LOG(INFO)<<"Initializing mapper 2";
        EMVS::MapperEMVS mapper2(cam2, dsi_shape);
        mapper2.name="2";

        switch( FLAGS_process_method )
        {
        case 1:
        {
            // 1-3. Compute two DSIs (one for each camera) and fuse them
            process_1(cam0,cam1,cam2, trajectory0,trajectory1,trajectory2, events0,events1,events2, opts_depth_map,dsi_shape, mapper_fused, mapper0, mapper1, mapper2, FLAGS_out_path, FLAGS_ts, FLAGS_stereo_fusion);
            break;
        }
        case 2:
        {
            // 1-3. Split events into sub-intervals, compute fused DSIs per sub-interval, and fuse them into a single DSI
            process_2(cam0,cam1,trajectory0,trajectory1,events0,events1,opts_depth_map,dsi_shape,FLAGS_num_intervals, mapper_fused, mapper_fused_camera_time, FLAGS_out_path, FLAGS_ts, FLAGS_stereo_fusion, FLAGS_temporal_fusion);
            break;
        }
        case 5:
        {
            // Shuffle event sub-intervals before fusing
            process_5(cam0,cam1,trajectory0,trajectory1,events0,events1,opts_depth_map,dsi_shape,FLAGS_num_intervals, mapper_fused, FLAGS_out_path, FLAGS_ts, FLAGS_stereo_fusion, FLAGS_temporal_fusion);
            break;
        }
        }

        cv::Mat depth_map_dense;
        mapper_fused.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map, depth_map_dense);
        //    saveDepthMaps(depth_map, confidence_map, semidense_mask, depth_map_dense, dsi_shape.min_depth_, dsi_shape.max_depth_, std::string("fused"), FLAGS_out_path + FLAGS_ts);

        // 4. Convert semi-dense depth map to point cloud
        EMVS::OptionsPointCloud opts_pc;
        opts_pc.radius_search_ = FLAGS_radius_search;
        opts_pc.min_num_neighbors_ = FLAGS_min_num_neighbors;

        EMVS::PointCloud::Ptr pc (new EMVS::PointCloud);
        mapper_fused.getPointcloud(depth_map, semidense_mask, opts_pc, pc);

        // Save point cloud to disk
        pcl::io::savePCDFileASCII (FLAGS_out_path+"pointcloud.pcd", *pc);
        LOG(INFO) << "Saved " << pc->points.size () << " data points to pointcloud.pcd";

        if (FLAGS_late_fusion){

            mapper0.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map, depth_map_dense);
            // 4. Convert semi-dense depth map to point cloud
            //          EMVS::OptionsPointCloud opts_pc;
            opts_pc.radius_search_ = FLAGS_radius_search;
            opts_pc.min_num_neighbors_ = FLAGS_min_num_neighbors;
            EMVS::PointCloud::Ptr pc0 (new EMVS::PointCloud);
            mapper_fused.getPointcloud(depth_map, semidense_mask, opts_pc, pc0);
            // Save point cloud to disk
            pcl::io::savePCDFileASCII (FLAGS_out_path+"pointcloud0.pcd", *pc0);
            LOG(INFO) << "Saved " << pc0->points.size () << " data points to pointcloud.pcd";

            mapper1.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map, depth_map_dense);
            // 4. Convert semi-dense depth map to point cloud
            //          EMVS::OptionsPointCloud opts_pc;
            opts_pc.radius_search_ = FLAGS_radius_search;
            opts_pc.min_num_neighbors_ = FLAGS_min_num_neighbors;
            EMVS::PointCloud::Ptr pc1 (new EMVS::PointCloud);
            mapper_fused.getPointcloud(depth_map, semidense_mask, opts_pc, pc1);
            // Save point cloud to disk
            pcl::io::savePCDFileASCII (FLAGS_out_path+"pointcloud1.pcd", *pc1);
            LOG(INFO) << "Saved " << pc1->points.size () << " data points to pointcloud.pcd";

            EMVS::PointCloud::Ptr pc_concat(new EMVS::PointCloud);
            pcl::concatenate(*pc0, *pc1, *pc_concat);
            pcl::io::savePCDFileASCII (FLAGS_out_path+"pointcloud_concat.pcd", *pc_concat);
            LOG(INFO) << "Saved " << pc_concat->points.size () << " data points to pointcloud.pcd";
        }
    }
}
