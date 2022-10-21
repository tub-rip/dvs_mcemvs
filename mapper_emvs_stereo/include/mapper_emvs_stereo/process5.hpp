/*
* \file process5.hpp
* \brief header file for Alg 2 + shuffling
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

#include <mapper_emvs_stereo/mapper_emvs_stereo.hpp>
#include <vector>

// Compute both DSIs, fuse by harmonic mean, then extract semi-dense depth map
void process_5(const image_geometry::PinholeCameraModel& cam0,
               const image_geometry::PinholeCameraModel& cam1,
               const LinearTrajectory& trajectory0,
               const LinearTrajectory& trajectory1,
               const std::vector<dvs_msgs::Event>& events0,
               const std::vector<dvs_msgs::Event>& events1,
               const EMVS::OptionsDepthMap& opts_depth_map,
               const EMVS::ShapeDSI& dsi_shape,
               const int num_subintervals,
               EMVS::MapperEMVS& mapper_fused,
               const std::string& out_path,
               double ts,
               int stereo_fusion,
               int temporal_fusion
);
