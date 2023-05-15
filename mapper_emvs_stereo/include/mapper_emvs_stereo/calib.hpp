/*
* \file calib.hpp
* \brief header for calibration functions
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

#include <mapper_emvs_stereo/geometry_utils.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <yaml-cpp/yaml.h>
#include <unsupported/Eigen/MatrixFunctions>

// Calibration functions TEMPORARY. They should be read from input files

void loadCalibInfo(const std::string &cameraSystemDir, bool bPrintCalibInfo);


void get_camera_calib_yaml_m3ed(image_geometry::PinholeCameraModel& cam0,
                                 image_geometry::PinholeCameraModel& cam1,
                                 Eigen::Matrix4d& mat4_1_0,
                                 Eigen::Matrix4d& mat4_hand_eye,
                                 std::string calib_path);

void get_camera_calib_dsec_yaml(image_geometry::PinholeCameraModel& cam0,
                                image_geometry::PinholeCameraModel& cam1,
                                Eigen::Matrix4d& mat4_1_0,
                                Eigen::Matrix4d& mat4_hand_eye,
                                std::string calib_path,
                                std::string mocap_calib_path);

void get_camera_calib_dsec_zurich04a(image_geometry::PinholeCameraModel& cam0,
                                     image_geometry::PinholeCameraModel& cam1,
                                     Eigen::Matrix4d& matR_L,
                                     Eigen::Matrix4d& mat4_hand_eye,
                                     std::string calib_path);

void get_camera_calib_dsec_interlaken00b(image_geometry::PinholeCameraModel& cam0,
                                         image_geometry::PinholeCameraModel& cam1,
                                         Eigen::Matrix4d& matR_L,
                                         Eigen::Matrix4d& mat4_hand_eye,
                                         std::string calib_path);
void get_camera_calib_yaml(image_geometry::PinholeCameraModel& cam0,
                           image_geometry::PinholeCameraModel& cam1,
                           Eigen::Matrix4d& mat4_1_0,
                           Eigen::Matrix4d& mat4_hand_eye,
                           std::string calib_path);

void get_camera_calib_json(image_geometry::PinholeCameraModel& cam0,
                           image_geometry::PinholeCameraModel& cam1,
                           Eigen::Matrix4d& mat4_1_0,
                           Eigen::Matrix4d& mat4_hand_eye,
                           std::string calib_pathA,
                           std::string calib_pathB);

void get_camera_calib_slider(image_geometry::PinholeCameraModel& cam0,
                             image_geometry::PinholeCameraModel& cam1,
                             Eigen::Matrix4d& mat4_1_0,
                             Eigen::Matrix4d& mat4_hand_eye,
                             std::string calib_path);

void get_camera_calib_hkust(image_geometry::PinholeCameraModel& cam0,
                            image_geometry::PinholeCameraModel& cam1,
                            Eigen::Matrix4d& mat4_1_0,
                            Eigen::Matrix4d& mat4_hand_eye,
                            std::string calib_path);

void get_camera_calib_evimo2(image_geometry::PinholeCameraModel& cam0,
                             image_geometry::PinholeCameraModel& cam1,
                             image_geometry::PinholeCameraModel& cam2,
                             Eigen::Matrix4d& mat4_1_0,
                             Eigen::Matrix4d& mat4_2_0,
                             Eigen::Matrix4d& mat4_hand_eye,
                             std::string calib_path);

void get_camera_calib_yaml_mvsec(image_geometry::PinholeCameraModel& cam0,
                                 image_geometry::PinholeCameraModel& cam1,
                                 Eigen::Matrix4d& mat4_1_0,
                                 Eigen::Matrix4d& mat4_hand_eye,
                                 std::string calib_path);

void get_camera_calib_ESIM(image_geometry::PinholeCameraModel& cam0, 
                           image_geometry::PinholeCameraModel& cam1,
                           Eigen::Matrix4d& mat4_1_0,
                           Eigen::Matrix4d& mat4_hand_eye);

// rpg_DAVIS_stereo dataset (Joey ECCV'18)
void get_camera_calib_ECCV18(image_geometry::PinholeCameraModel& cam0, 
                             image_geometry::PinholeCameraModel& cam1,
                             Eigen::Matrix4d& mat4_1_0,
                             Eigen::Matrix4d& mat4_hand_eye);

// Samsung DVS Gen3 stereo
void get_camera_calib_DVS_Gen3(image_geometry::PinholeCameraModel& cam0, 
                               image_geometry::PinholeCameraModel& cam1,
                               Eigen::Matrix4d& mat4_1_0,
                               Eigen::Matrix4d& mat4_hand_eye);
