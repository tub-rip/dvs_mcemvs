/*
* \file data_loading.hpp
* \brief header for functions to load events and poses from ROSBags
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


#pragma once

#include <string>
#include <map>
#include <ros/time.h>
#include <mapper_emvs_stereo/geometry_utils.hpp>

#include <dvs_msgs/EventArray.h>
#include <camera_info_manager/camera_info_manager.h>

namespace data_loading {

void parse_rosbag(const std::string &rosbag,
                  std::vector<dvs_msgs::Event>& events_,
                  std::map<ros::Time, geometry_utils::Transformation>& poses_,
                  sensor_msgs::CameraInfo& camera_info_msg,
                  const std::string& event_topic,
                  const std::string& camera_info_topic,
                  const std::string& pose_topic,
                  const double tmin,
                  const double tmax,
                  const double events_offset);

// MVSEC has separated files for data and ground truth
void parse_rosbag(const std::string &rosbag,
                  std::vector<dvs_msgs::Event>& events_,
                  sensor_msgs::CameraInfo& camera_info_msg,
                  const std::string& event_topic,
                  const std::string& camera_info_topic,
                  const double tmin,
                  const double tmax,
                  const double events_offset);

void parse_rosbag_gt(const std::string &rosbag,
                  std::map<ros::Time, geometry_utils::Transformation>& poses_,
                  const std::string& pose_topic,
                  const double tmin,
                  const double tmax);

} // namespace data_loading
