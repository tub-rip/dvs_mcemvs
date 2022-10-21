/*
* \file utils.hpp
* \brief header file for utility functions like saving depth and confidence maps
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

#include <string.h>
#include <opencv2/core.hpp>
#include <mapper_emvs_stereo/mapper_emvs_stereo.hpp>

void saveDepthMaps(const cv::Mat& depth_map, 
                   const cv::Mat& confidence_map, 
                   const cv::Mat& semidense_mask,
                   const cv::Mat& depth_map_dense,
                   const float min_depth,
                   const float max_depth,
                   const std::string& suffix,
                   const std::string& out_path);

void saveDepthMaps(const cv::Mat& depth_map, 
                   const cv::Mat& confidence_map, 
                   const cv::Mat& semidense_mask,
                   const float min_depth,
                   const float max_depth,
                   const std::string& suffix,
                   const std::string& out_path);

// Harmonic mean of two DSIs in mapper objects
void fuseDSIs_HarmonicMean(const EMVS::MapperEMVS& mapper0,
                           const EMVS::MapperEMVS& mapper1,
                           EMVS::MapperEMVS& mapper_fused);

// Sum (e.g., arithmetic mean) of two DSIs in mapper objects
void fuseDSIs_Sum(const EMVS::MapperEMVS& mapper0,
                  const EMVS::MapperEMVS& mapper1,
                  EMVS::MapperEMVS& mapper_fused);


double computeFocus_MeanSquare(const cv::Mat& img);
double computeFocus_Variance(const cv::Mat& img);


void fuseDSIs_HarmonicMeanOfLocalFocus(const EMVS::MapperEMVS& mapper0,
                                       const EMVS::MapperEMVS& mapper1,
                                       const image_geometry::PinholeCameraModel& cam0,
                                       const image_geometry::PinholeCameraModel& cam1,
                                       const EMVS::ShapeDSI& dsi_shape,
                                       const int focus_method, // 0 for variance, 1 for MS
                                       EMVS::MapperEMVS& mapper_focus_fused);

void accumulateEvents(const std::vector<dvs_msgs::Event>& events,
                      const bool use_polarity,
                      cv::Mat& img);

void evaluateDepthMap(cv::Mat depth, cv::Mat gt);
