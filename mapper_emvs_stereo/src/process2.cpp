/*
* \file process2.cpp
* \brief process events using Alg 2
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

#include <mapper_emvs_stereo/process2.hpp>
#include <mapper_emvs_stereo/utils.hpp>

#include <opencv2/highgui.hpp>
#include <glog/logging.h>
#include <sstream>

//#define TIMING_LOOP

// Alg 2: Fusion across cameras and time
// Divide event stream into sub-intervals and generate DSIs. Fuse across cameras, then across time.
// Finally, also do the converse: first fuse across time, then across cameras.
void process_2(const image_geometry::PinholeCameraModel& cam0,
               const image_geometry::PinholeCameraModel& cam1,
               const LinearTrajectory& trajectory0,
               const LinearTrajectory& trajectory1,
               const std::vector<dvs_msgs::Event>& events0,
               const std::vector<dvs_msgs::Event>& events1,
               const EMVS::OptionsDepthMap& opts_depth_map,
               const EMVS::ShapeDSI& dsi_shape,
               const int num_subintervals,
               EMVS::MapperEMVS& mapper_fused,
               EMVS::MapperEMVS& mapper_fused_camera_time,
               const std::string& out_path,
               double ts,
               int stereo_fusion,
               int temporal_fusion
               )
{
  // Initialize mappers
  const unsigned int num_events_per_subinterval[2] = {static_cast<unsigned int>(events0.size()) / num_subintervals,
                                                      static_cast<unsigned int>(events1.size()) / num_subintervals};

  // Initialize mappers
  EMVS::MapperEMVS mapper0(cam0, dsi_shape);
  EMVS::MapperEMVS mapper1(cam1, dsi_shape);
  EMVS::MapperEMVS mapper_fused_subinterval(cam0, dsi_shape);
  EMVS::MapperEMVS mapper_fused_left(cam0, dsi_shape);
  EMVS::MapperEMVS mapper_fused_right(cam0, dsi_shape);

  geometry_utils::Transformation T_rv_w;
  double t_mid;

  // Set the location of the DSI (reference view) in the middle of the trajectory of the left camera
  //  if(ts <0)
  //    {
  //      geometry_utils::Transformation T0, T1;
  //      ros::Time t0, t1;
  //      trajectory0.getFirstControlPose(&T0, &t0);
  //      trajectory0.getLastControlPose(&T1, &t1);
  //      t_mid = 0.5 * (t0.toSec() + t1.toSec());
  //      LOG(INFO) << "Setting DSI reference at middle of left camera trajectory: " << t_mid;
  //    } // Set DSI reference at provided timestamp ts
  //  else
  //    {
  t_mid = ts;
  LOG(INFO) << "Setting DSI reference at specific timestamp: " << t_mid;
  //    }

  // set up prefix including output path
  std::stringstream ss;
  ss << out_path << std::fixed << std::setprecision(9) << std::setfill('0') << std::setw(13) << t_mid;

  geometry_utils::Transformation T_w_rv;
  trajectory0.getPoseAt(ros::Time(t_mid), T_w_rv);
  T_rv_w = T_w_rv.inverse();

  cv::Mat depth_map, confidence_map, semidense_mask;

  LOG(INFO) << "Computing DSI for the cameras";
  LOG(INFO) << "Left : Number of events per sub-interval = " << num_events_per_subinterval[0] << " events";
  LOG(INFO) << "Right: Number of events per sub-interval = " << num_events_per_subinterval[1] << " events";

  // Fuse the DSIs
  mapper_fused.dsi_.resetGrid();
  unsigned int idx_first_ev[2] = {0, 0};
#ifdef TIMING_LOOP
  int nloops = 100;
#endif
  std::chrono::high_resolution_clock::time_point t_start, t_end = std::chrono::high_resolution_clock::now();
  auto fusion_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_end).count();

  for (unsigned int k=0; k < num_subintervals; k++)
    {
      mapper0.dsi_.resetGrid();
      mapper1.dsi_.resetGrid();
      mapper_fused_subinterval.dsi_.resetGrid();
      {// Left camera:
        // Approximation: this selection does not guarantee that events belong to the same time interval on both cameras
        std::vector<dvs_msgs::Event> events_subset = std::vector<dvs_msgs::Event>(events0.begin() + idx_first_ev[0],
            events0.begin() + idx_first_ev[0] + num_events_per_subinterval[0]);
        idx_first_ev[0] += num_events_per_subinterval[0];

        {// Display accumulted events to visualize the amount of intra-camera shift/baseline
          cv::Size full_resolution = cam0.fullResolution();
          cv::Mat event_image = cv::Mat(full_resolution,CV_8UC1);
          accumulateEvents(events_subset,true,event_image);
          std::stringstream filename;
          filename << "events_0_" << std::setfill('0') << std::setw(3) << k << ".png";
          cv::imwrite(out_path+filename.str(),event_image);
        }

        // Back-project events into the DSI
        mapper0.evaluateDSI(events_subset, trajectory0, T_rv_w);
        LOG(INFO) << "Left : Sub-interval " << k << "  Mean square = " << mapper0.dsi_.computeMeanSquare();

        if (!opts_depth_map.full_sequence) {
            mapper0.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);
            std::stringstream ss_suffix;
            ss_suffix << "0_" << std::setfill('0') << std::setw(3) << k;
            saveDepthMaps(depth_map, confidence_map, semidense_mask, dsi_shape.min_depth_, dsi_shape.max_depth_, ss_suffix.str(), ss.str());
          }
      }

      {// Right camera:
        // Approximation: this selection does not guarantee that events belong to the same time interval on both cameras
        std::vector<dvs_msgs::Event> events_subset = std::vector<dvs_msgs::Event>(events1.begin() + idx_first_ev[1],
            events1.begin() + idx_first_ev[1] + num_events_per_subinterval[1]);
        idx_first_ev[1] += num_events_per_subinterval[1];

        {// Display accumulted events to visualize the amount of intra-camera shift/baseline
          cv::Size full_resolution = cam1.fullResolution();
          cv::Mat event_image = cv::Mat(full_resolution,CV_8UC1);
          accumulateEvents(events_subset,true,event_image);
          std::stringstream filename;
          filename << "events_1_" << std::setfill('0') << std::setw(3) << k << ".png";
          cv::imwrite(out_path+filename.str(),event_image);
        }

        // Back-project events into the DSI
        mapper1.evaluateDSI(events_subset, trajectory1, T_rv_w);
        LOG(INFO) << "Right: Sub-interval " << k << "  Mean square = " << mapper1.dsi_.computeMeanSquare();

        if (!opts_depth_map.full_sequence) {
            mapper1.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);
            std::stringstream ss_suffix;
            ss_suffix << "1_" << std::setfill('0') << std::setw(3) << k;
            saveDepthMaps(depth_map, confidence_map, semidense_mask, dsi_shape.min_depth_, dsi_shape.max_depth_, ss_suffix.str(), ss.str());
          }

      }

      // Initialize stereo mapper for a single subinterval
      mapper_fused_subinterval.dsi_.resetGrid();
      mapper_fused_subinterval.dsi_.addTwoGrids(mapper0.dsi_);

      // Fused DSI across cameras for a single sub-interval
      t_start = std::chrono::high_resolution_clock::now();
#ifdef TIMING_LOOP
      for (int i=1; i<=nloops; i++){
#endif
          switch(stereo_fusion){
            case 1:
              mapper_fused_subinterval.dsi_.minTwoGrids(mapper1.dsi_);
              break;
            case 2:
              mapper_fused_subinterval.dsi_.harmonicMeanTwoGrids(mapper1.dsi_);
              break;
            case 3:
              mapper_fused_subinterval.dsi_.geometricMeanTwoGrids(mapper1.dsi_);
              break;
            case 4:
              mapper_fused_subinterval.dsi_.arithmeticMeanTwoGrids(mapper1.dsi_);
              break;
            case 5:
              mapper_fused_subinterval.dsi_.rmsTwoGrids(mapper1.dsi_);
              break;
            case 6:
              mapper_fused_subinterval.dsi_.maxTwoGrids(mapper1.dsi_);
              break;
            default:
              LOG(INFO) << "Improper fusion method selected";
              return;
            }
#ifdef TIMING_LOOP
        }
#endif
      t_end = std::chrono::high_resolution_clock::now();
      fusion_duration+=std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count();

      if (!opts_depth_map.full_sequence) {
          // Save depth map from stereo DSI for a single interval
          mapper_fused_subinterval.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);
          {
            std::stringstream ss_suffix;
            ss_suffix << "fused_" << std::setfill('0') << std::setw(3) << k;
            saveDepthMaps(depth_map, confidence_map, semidense_mask, dsi_shape.min_depth_, dsi_shape.max_depth_, ss_suffix.str(), ss.str());
          }
        }

      // Fuse DSIs across time
      t_start = std::chrono::high_resolution_clock::now();
#ifdef TIMING_LOOP
      for (int i=1; i<=nloops; i++){
#endif
          switch (temporal_fusion){
            case 1:
              break;
            case 2:
              // HM
              mapper_fused_left.dsi_.addInverseOfTwoGrids(mapper0.dsi_);
              mapper_fused_right.dsi_.addInverseOfTwoGrids(mapper1.dsi_);
              mapper_fused.dsi_.addInverseOfTwoGrids(mapper_fused_subinterval.dsi_);
              if (k==num_subintervals-1){
                  mapper_fused_left.dsi_.computeHMfromSumOfInv(num_subintervals);
                  mapper_fused_right.dsi_.computeHMfromSumOfInv(num_subintervals);
                  mapper_fused.dsi_.computeHMfromSumOfInv(num_subintervals);
                }
              break;
            case 3:
              break;
            case 4:
              // AM
              mapper_fused_left.dsi_.addTwoGrids(mapper0.dsi_);
              mapper_fused_right.dsi_.addTwoGrids(mapper1.dsi_);
              mapper_fused.dsi_.addTwoGrids(mapper_fused_subinterval.dsi_);
              if (k==num_subintervals-1){
                  mapper_fused_left.dsi_.computeAMfromSum(num_subintervals);
                  mapper_fused_right.dsi_.computeAMfromSum(num_subintervals);
                  mapper_fused.dsi_.computeAMfromSum(num_subintervals);
                }
              break;
            case 5:
              break;
            case 6:
              break;
            }
#ifdef TIMING_LOOP
        }
#endif
      t_end = std::chrono::high_resolution_clock::now();
      fusion_duration+= (std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count())/3;

    }

  LOG(INFO) << "Time taken to fuse across space and time "<< fusion_duration <<" ms";
  LOG(INFO) << "Mean square = " << mapper_fused.dsi_.computeMeanSquare();


  if (!opts_depth_map.full_sequence) {
      // Fused DSIs (using harmonic mean).
      mapper_fused_left.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);
      saveDepthMaps(depth_map, confidence_map, semidense_mask, dsi_shape.min_depth_, dsi_shape.max_depth_, std::string("left_temporal_" + std::to_string(temporal_fusion)), ss.str());
      mapper_fused_right.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);
      saveDepthMaps(depth_map, confidence_map, semidense_mask, dsi_shape.min_depth_, dsi_shape.max_depth_, std::string("right_temporal_" + std::to_string(temporal_fusion)), ss.str());
    }
  mapper_fused.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);
  saveDepthMaps(depth_map, confidence_map, semidense_mask, dsi_shape.min_depth_, dsi_shape.max_depth_, std::string("stereo_temporal_" + std::to_string(temporal_fusion)), ss.str());

  // Now fusing all sub-intervals along time first, then across cameras (converse of above process)
  mapper_fused_camera_time.dsi_.addTwoGrids(mapper_fused_left.dsi_);
  switch(stereo_fusion){
    case 1:
      mapper_fused_camera_time.dsi_.minTwoGrids(mapper_fused_right.dsi_);
      break;
    case 2:
      mapper_fused_camera_time.dsi_.harmonicMeanTwoGrids(mapper_fused_right.dsi_);
      break;
    case 3:
      mapper_fused_camera_time.dsi_.arithmeticMeanTwoGrids(mapper_fused_right.dsi_);
      break;
    case 4:
      mapper_fused_camera_time.dsi_.geometricMeanTwoGrids(mapper_fused_right.dsi_);
      break;
    case 5:
      mapper_fused_camera_time.dsi_.rmsTwoGrids(mapper_fused_right.dsi_);
      break;
    case 6:
      mapper_fused_camera_time.dsi_.maxTwoGrids(mapper_fused_right.dsi_);
      break;
    default:
      LOG(INFO) << "Improper stereo fusion method selected";
      return;
    }

  if(opts_depth_map.save_dsi) {
      // Write the DSI (3D voxel grid) to disk
      mapper_fused_left.dsi_.writeGridNpy(std::string(out_path + "dsi_fused_0_temporalfusion.npy").c_str());
      mapper_fused_right.dsi_.writeGridNpy(std::string(out_path + "dsi_fused_1_temporalfusion.npy").c_str());
      mapper_fused.dsi_.writeGridNpy(std::string(out_path + "dsi_stereo_temporalfusion.npy").c_str());
      mapper_fused_camera_time.dsi_.writeGridNpy(std::string(out_path + "dsi_stereo_temporalfusion_camera_time.npy").c_str());
    }

  mapper_fused_camera_time.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);
  saveDepthMaps(depth_map, confidence_map, semidense_mask, dsi_shape.min_depth_, dsi_shape.max_depth_, std::string("stereo_temporal_camera_time" + std::to_string(temporal_fusion)), ss.str());

}
