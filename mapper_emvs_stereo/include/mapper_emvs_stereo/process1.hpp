/*
* \file process1.hpp
* \brief header file for Alg 1
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
void process_1(
    const image_geometry::PinholeCameraModel& cam0,
    const image_geometry::PinholeCameraModel& cam1,
    const image_geometry::PinholeCameraModel& cam2,
    const LinearTrajectory& trajectory0,
    const LinearTrajectory& trajectory1,
    const LinearTrajectory& trajectory2,
    const std::vector<dvs_msgs::Event>& events0,
    const std::vector<dvs_msgs::Event>& events1,
    const std::vector<dvs_msgs::Event>& events2,
    const EMVS::OptionsDepthMap& opts_depth_map,
    const EMVS::ShapeDSI& dsi_shape,
    EMVS::MapperEMVS& mapper_fused,
    EMVS::MapperEMVS& mapper0,
    EMVS::MapperEMVS& mapper1,
    EMVS::MapperEMVS& mapper2,
    const std::string& out_path,
    double ts,
    int fusion_method
    );
