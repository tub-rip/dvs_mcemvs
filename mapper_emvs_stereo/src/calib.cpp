/*
* \file calib.cpp
* \brief calibration functions
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

#include <mapper_emvs_stereo/calib.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <vicon/Subject.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include<opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

// Calibration functions

//Load calib params from yaml when cam extrinsics are provided wrt base B (hand)
void get_camera_calib_yaml(image_geometry::PinholeCameraModel& cam0,
                           image_geometry::PinholeCameraModel& cam1,
                           Eigen::Matrix4d& mat4_1_0,
                           Eigen::Matrix4d& mat4_hand_eye,
                           std::string calib_path)
{
    sensor_msgs::CameraInfo camera_info;
    YAML::Node calibInfo = YAML::LoadFile(calib_path);
    YAML::Node cameras = calibInfo["cameras"];
    YAML::Node cameraL = cameras[0]["camera"];
    camera_info.height = cameraL["image_height"].as<int>();
    camera_info.width = cameraL["image_width"].as<int>();
    std::vector<double> intrinsics = cameraL["intrinsics"]["data"].as<std::vector<double>>();
    camera_info.K = {intrinsics[0], 0., intrinsics[2],
                     0., intrinsics[1], intrinsics[3],
                     0., 0., 1.};
    std::string distortionType = cameraL["distortion"]["type"].as<std::string>();
    if (distortionType == "none"){
        camera_info.distortion_model = "plumb_bob";
        camera_info.D = {0., 0., 0., 0., 0.}; // plumb_bob (rad-tan)
    }
    camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    camera_info.P = {intrinsics[0],  0., intrinsics[2], 0,
                     0., intrinsics[1],  intrinsics[3], 0,
                     0.,  0.,  1., 0};
    cam0.fromCameraInfo(camera_info);
    cam1.fromCameraInfo(camera_info);

    // Extrinsics
    Eigen::Matrix4d T_B_left, T_B_right;
    T_B_left = Eigen::Matrix<double,4,4,Eigen::RowMajor>(cameras[0]["T_B_C"]["data"].as<std::vector<double>>().data());
    T_B_right = Eigen::Matrix<double,4,4,Eigen::RowMajor>(cameras[1]["T_B_C"]["data"].as<std::vector<double>>().data());
    mat4_1_0 = T_B_right.inverse()*T_B_left;
    //    mat4_hand_eye = T_B_left;
    mat4_hand_eye = Eigen::Matrix4d::Identity(4,4);
    //    LOG(INFO)<<"extrinsic: "<<mat4_1_0;
    //    LOG(INFO)<<"hand eye: "<<mat4_hand_eye;
}


void get_camera_calib_json(image_geometry::PinholeCameraModel& cam0,
                           image_geometry::PinholeCameraModel& cam1,
                           Eigen::Matrix4d& mat4_1_0,
                           Eigen::Matrix4d& mat4_hand_eye,
                           std::string camera_calib_path,
                           std::string mocap_calib_path)
{
    cv::FileStorage fs(camera_calib_path, cv::FileStorage::READ);
    std::vector<sensor_msgs::CameraInfo> camera_info(2);
    std::vector<Eigen::Matrix4d> T_imu_cam(2);

    for(int i=0; i<2; i++){

        camera_info[i].height = (double)fs["value0"]["resolution"][i+2][1];
        camera_info[i].width = (double)fs["value0"]["resolution"][i+2][0];

        auto intrinsics = fs["value0"]["intrinsics"][i+2]["intrinsics"];
        camera_info[i].K = {intrinsics["fx"], 0., intrinsics["cx"],
                            0., intrinsics["fy"], intrinsics["cy"],
                            0., 0., 1.};

        std::string distortionType = fs["value0"]["intrinsics"][i+2]["camera_type"];
        if (distortionType == "kb4"){
            camera_info[i].distortion_model = "fisheye";
            camera_info[i].D = {intrinsics["k1"], intrinsics["k2"], intrinsics["k3"], intrinsics["k4"]}; // plumb_bob (rad-tan)
        }

        camera_info[i].R = {1, 0, 0,
                            0, 1, 0,
                            0, 0, 1};

        //        cv::Mat K_temp(3, 3, CV_64F, camera_info[i].K.data());
        //        cv::Mat P_temp;
        //        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_temp, camera_info[i].D, cv::Size(camera_info[i].width, camera_info[i].height), cv::Mat::eye(3,3,CV_32FC1), P_temp);
        //        for (int r = 0; r<3; r++){
        //            for (int c=0; c<4; c++){
        //                camera_info[i].P[r*4+c]=P_temp.at<double>(r, c);
        //            }
        //        }

        camera_info[i].P = {0.8*(float)intrinsics["fx"], 0., intrinsics["cx"], 0,
                            0., 0.8*(float)intrinsics["fy"], intrinsics["cy"], 0,
                            0., 0., 1., 0.};

        auto extrinsics = fs["value0"]["T_imu_cam"][i+2];
        Eigen::Quaterniond q_imu_cam((double)extrinsics["qw"],
                (double)extrinsics["qx"],
                (double)extrinsics["qy"],
                (double)extrinsics["qz"]);
        Eigen::Vector3d t_imu_cam((double)extrinsics["px"],
                (double)extrinsics["py"],
                (double)extrinsics["pz"]);
        Eigen::Matrix3d R = q_imu_cam.toRotationMatrix();
        T_imu_cam[i].block(0, 0, 3, 3) << R;
        T_imu_cam[i].block(0, 3, 3, 1) << t_imu_cam;
        T_imu_cam[i].row(3) << 0, 0, 0, 1;
    }
    cam0.fromCameraInfo(camera_info[0]);
    camera_info[1].P = camera_info[0].P;
    cam1.fromCameraInfo(camera_info[1]);

    //    LOG(INFO)<< "T_imu_cam0"<<T_imu_cam[0];
    //    LOG(INFO)<< "T_imu_cam1"<<T_imu_cam[1];

    mat4_1_0 = T_imu_cam[1].inverse()*T_imu_cam[0];

    if (mocap_calib_path.empty()==0){
        cv::FileStorage fs_mocap(mocap_calib_path, cv::FileStorage::READ);
        LOG(INFO) << "Mocap calib being used";
        auto extrinsics = fs_mocap["value0"]["T_imu_marker"];
        Eigen::Quaterniond q_imu_m(extrinsics["qw"],
                extrinsics["qx"],
                extrinsics["qy"],
                extrinsics["qz"]);
        Eigen::Vector3d t_imu_m(extrinsics["px"],
                extrinsics["py"],
                extrinsics["pz"]);
        Eigen::Matrix3d R = q_imu_m.toRotationMatrix();
        Eigen::Matrix4d T_imu_m;
        T_imu_m.block(0, 0, 3, 3) << R;
        T_imu_m.block(0, 3, 3, 1) << t_imu_m;
        T_imu_m.row(3) << 0, 0, 0, 1;
        mat4_hand_eye =T_imu_m.inverse() * T_imu_cam[0];
    }
    else {
        mat4_hand_eye = T_imu_cam[0];
    }

    LOG(INFO)<<"extrinsic: "<<mat4_1_0.format(Eigen::FullPrecision);
    LOG(INFO)<<"hand eye: "<<mat4_hand_eye.format(Eigen::FullPrecision);
}



void get_camera_calib_dsec_yaml(image_geometry::PinholeCameraModel& cam0,
                                image_geometry::PinholeCameraModel& cam1,
                                Eigen::Matrix4d& matR_L,
                                Eigen::Matrix4d& mat4_hand_eye,
                                std::string calib_path,
                                std::string mocap_calib_path)
{
    std::vector<sensor_msgs::CameraInfo> camera_info(2);

    YAML::Node calibInfo = YAML::LoadFile(calib_path);
    int event_cam_id[2] = {0, 3};

    //Intrinsics
    for (int i=0; i<=1; i++){
        YAML::Node camera = calibInfo["intrinsics"]["cam"+std::to_string(event_cam_id[i])];
        assert(camera["camera_type"] == "event");
        camera_info[i].height = camera["resolution"][1].as<int>();
        camera_info[i].width = camera["resolution"][0].as<int>();
        camera_info[i].K = {camera["camera_matrix"][0].as<double>(), 0., camera["camera_matrix"][2].as<double>(),
                            0., camera["camera_matrix"][1].as<double>(), camera["camera_matrix"][3].as<double>(),
                            0., 0., 1.};

        std::string distortionType = camera["distortion_model"].as<std::string>();
        if (distortionType == "none"){
            camera_info[i].distortion_model = "plumb_bob";
            camera_info[i].D = {0., 0., 0., 0., 0.}; // plumb_bob (rad-tan)
        }
        else if(distortionType == "radtan"){
            camera_info[i].distortion_model="plumb_bob";
            camera_info[i].D = {camera["distortion_coeffs"][0].as<double>(), camera["distortion_coeffs"][1].as<double>(), camera["distortion_coeffs"][2].as<double>(), camera["distortion_coeffs"][3].as<double>()};
        }

        //Ignore rectification rotation, work on unrectified images
        camera_info[i].R = {1, 0, 0,
                            0, 1, 0,
                            0, 0, 1};

        cv::Mat K_temp(3, 3, CV_64F, camera_info[i].K.data());
        cv::Mat P_temp = cv::getOptimalNewCameraMatrix(K_temp, camera_info[i].D, cv::Size(camera_info[i].width, camera_info[i].height), 0);
        for (int r = 0; r<3; r++){
            for (int c=0; c<4; c++){
                camera_info[i].P[r*4+c]=P_temp.at<double>(r, c);
            }
        }
    }

    cam0.fromCameraInfo(camera_info[0]);
    camera_info[1].P = camera_info[0].P;
    cam1.fromCameraInfo(camera_info[1]);

    // Extrinsics
    const YAML::Node ext = calibInfo["extrinsics"];
    Eigen::Matrix4d T_32, T_21, T_10, T_lidar_camRect1, T_rect1_1;

    for (int r=0; r<4; r++){
        for (int c=0; c<4; c++){
            T_32(r, c) = ext["T_32"][r][c].as<double>();
        }
    }

    for (int r=0; r<4; r++){
        for (int c=0; c<4; c++){
            T_21(r, c) = ext["T_21"][r][c].as<double>();
        }
    }

    for (int r=0; r<4; r++){
        for (int c=0; c<4; c++){
            T_10(r, c) = ext["T_10"][r][c].as<double>();
        }
    }

    for (int r=0; r<3; r++){
        for (int c=0; c<3; c++){
            T_rect1_1(r, c) = ext["R_rect1"][r][c].as<double>();
        }
    }
    T_rect1_1.col(3) << 0,0,0,0;
    T_rect1_1.row(3) << 0,0,0,1;

    const YAML::Node mocap = YAML::LoadFile(mocap_calib_path);
    for (int r=0; r<4; r++){
        for (int c=0; c<4; c++){
            T_lidar_camRect1(r, c) = mocap["T_lidar_camRect1"][r][c].as<double>();
        }
    }

    matR_L = T_32 * T_21 * T_10; //matR_L is mat3_0
    mat4_hand_eye = T_lidar_camRect1 * T_rect1_1 * T_10;
    LOG(INFO) << "matR_L: " <<matR_L;
    LOG(INFO) << "mat_hand_eye: " <<mat4_hand_eye;

}

void get_camera_calib_dsec_zurich04a(image_geometry::PinholeCameraModel& cam0,
                                     image_geometry::PinholeCameraModel& cam1,
                                     Eigen::Matrix4d& matR_L,
                                     Eigen::Matrix4d& mat4_hand_eye,
                                     std::string calib_path)
{
    sensor_msgs::CameraInfo camera_info;
    camera_info.height = 480;
    camera_info.width = 640;
    camera_info.K = {553.4686750102932, 0, 346.65339162053317,
                     0, 553.3994078799127, 216.52092103243012,
                     0, 0, 1};
    camera_info.distortion_model = "plumb_bob";
    camera_info.D = {-0.09356476362537607, 0.19445779814646236, 7.642434980998821e-05, 0.0019563864604273664};
    camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    cv::Mat K(3, 3, CV_64F, camera_info.K.data());
    cv::Mat P = cv::getOptimalNewCameraMatrix(K, camera_info.D, cv::Size(camera_info.width, camera_info.height), 0);
    for (int r=0; r<3; r++){
        for (int c=0; c<4; c++){
            camera_info.P[r*4+c]=P.at<double>(r, c);
        }
    }
    cam0.fromCameraInfo(camera_info);

    camera_info.K = {552.1819422959984, 0, 336.87432177064744,
                     0, 551.4454720096484, 226.32630571403274,
                     0, 0, 1};
    camera_info.D = {-0.09493681546997375, 0.2021148065491477, 0.0005821287651820125, 0.0014552921745527136};
    cam1.fromCameraInfo(camera_info);

    // Extrinsics
    Eigen::Matrix4d T_10, T_21, T_32, T_lidar_camRect1, T_rect1_1;
    T_10 <<     0.9997329831508507, 0.00994674446197701, 0.020857245142004693, -0.043722240320426424,
            -0.01003579267550241, 0.999940949009329, 0.004169095789442527, 0.0010155694745410755,
            -0.020814544570561252, -0.004377301558648307, 0.9997737713930034, -0.013372668558381158,
            0, 0, 0, 1;

    T_21 <<   0.9998379578286035, -0.017926384876108554, 0.0016440226264295469, -0.5092603987305321,
            0.017914084504235202, 0.9998135043384297, 0.007214022378586629, -0.0022179629729152214,
            -0.0017730373650056029, -0.007183402242479184, 0.9999726271607238, 0.0042971588717280644,
            0, 0, 0, 1;

    T_32 <<   0.9999876185667624, -0.0034167786978265787, -0.0036177806040117192, -0.046041759529914676,
            0.0033579259589126046, 0.9998639316478117, -0.016150619896091543, -0.0011068440180470077,
            0.0036724714325840242, 0.01613827168886575, 0.9998630251891839, 0.012672727774474509,
            0, 0, 0, 1;

    T_lidar_camRect1 << 0.006502250714427837, 0.0016414391549515739, 0.9999775129537399, 0.448,
            -0.9996294044397522, 0.026445536238290795, 0.006456577459882262, 0.255,
            -0.026434343477244382, -0.999648908012493, 0.0018127863517872211, -0.215,
            0, 0, 0, 1;

    T_rect1_1 <<   0.9998858610925897, -0.013510711178262034, -0.006762061119800281, 0,
            0.013535205789223095, 0.9999019509726164, 0.0035897974036225495, 0,
            0.00671289739037555, -0.0036809135568848755, 0.9999706935125713, 0,
            0, 0, 0, 1;

    matR_L = T_32 * T_21 * T_10; //matR_L is mat3_0
    mat4_hand_eye = T_lidar_camRect1 * T_rect1_1 * T_10;
    LOG(INFO) << "matR_L: " <<matR_L;
    LOG(INFO) << "mat_hand_eye: " <<mat4_hand_eye;

}


void get_camera_calib_dsec_interlaken00b(image_geometry::PinholeCameraModel& cam0,
                                         image_geometry::PinholeCameraModel& cam1,
                                         Eigen::Matrix4d& matR_L,
                                         Eigen::Matrix4d& mat4_hand_eye,
                                         std::string calib_path)
{
    sensor_msgs::CameraInfo camera_info;
    camera_info.height = 480;
    camera_info.width = 640;
    camera_info.K = {555.6627242364661, 0, 342.5725306057865,
                     0, 555.8306341927942, 215.26831427862848,
                     0, 0, 1};
    camera_info.distortion_model = "plumb_bob";
    camera_info.D = {-0.09094341408134071, 0.18339771556281387, -0.0006982341741678465, 0.00041396758898911876};
    camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    cv::Mat K(3, 3, CV_64F, camera_info.K.data());
    cv::Mat P = cv::getOptimalNewCameraMatrix(K, camera_info.D, cv::Size(camera_info.width, camera_info.height), 0);
    for (int r=0; r<3; r++){
        for (int c=0; c<4; c++){
            camera_info.P[r*4+c]=P.at<double>(r, c);
        }
    }
    cam0.fromCameraInfo(camera_info);
    camera_info.K = {553.800041834315, 0, 333.21860953836267,
                     0, 553.7026022383894, 226.01033624096638,
                     0, 0, 1};
    camera_info.D = {-0.09492592983896557, 0.20394312250370014, 0.00033282360055722797, -0.001101242451777801};
    cam1.fromCameraInfo(camera_info);

    // Extrinsics
    Eigen::Matrix4d T_10, T_21, T_32, T_lidar_camRect1, T_rect1_1;
    T_10 <<    0.9996874046885865, 0.009652146488870916, 0.023063585478994113, -0.04410263392688484,
            -0.009722042371104245, 0.9999484753460813, 0.0029203673010648615, 0.0005281285423087664,
            -0.023034209322743096, -0.0031436795631953228, 0.9997297347181744, -0.01229891454144492,
            0, 0, 0, 1;

    T_21 <<   0.9998543808844597, -0.01706309861700861, -0.00026017635946350924, -0.5094961871754736,
            0.017064416377671962, 0.9998338346058513, 0.00641162000174109, -0.002022496204233391,
            0.0001507310227716978, -0.006415126105036775, 0.9999794115066636, 0.005365297617411473,
            0, 0, 0, 1;

    T_32 <<   0.9999880111304372, -0.003533401537847065, -0.003390083916194203, -0.04551026028184807,
            0.003476600244706753, 0.9998558803824363, -0.016617211420558598, -0.001048727690114844,
            0.0034483106189848347, 0.016605226232405814, 0.999856177465359, 0.013554100781902953,
            0, 0, 0, 1;

    T_lidar_camRect1 << 0.01539728189227399, -0.0012823052573279758, 0.9998806325774878, 0.448,
            -0.9996610000153124, 0.020978176075891836, 0.015420803380972237, 0.255,
            -0.02099544614233234, -0.9997791115150167, -0.0009588636652390625, -0.215,
            0, 0, 0, 1;

    T_rect1_1 <<   0.9998572179847892, -0.013025778024398856, -0.010764420587133948, 0,
            0.013060715513432202, 0.9999096430275752, 0.003181743349841093, 0,
            0.01072200326407413, -0.0033218800890692088, 0.9999369998948329, 0,
            0, 0, 0, 1;

    matR_L = T_32 * T_21 * T_10; //matR_L is mat3_0
    mat4_hand_eye = T_lidar_camRect1 * T_rect1_1 * T_10;
    LOG(INFO) << "matR_L: " <<matR_L;
    LOG(INFO) << "mat_hand_eye: " <<mat4_hand_eye;

}



void get_camera_calib_slider(image_geometry::PinholeCameraModel& cam0,
                             image_geometry::PinholeCameraModel& cam1,
                             Eigen::Matrix4d& mat4_1_0,
                             Eigen::Matrix4d& mat4_hand_eye,
                             std::string calib_path)
{
    sensor_msgs::CameraInfo camera_info;
    camera_info.height = 180;
    camera_info.width = 240;
    camera_info.K = {198.9035679113487, 0, 139.8751842835105,
                     0, 198.8472302496314, 104.0170363461823,
                     0, 0, 1};
    camera_info.distortion_model = "plumb_bob";
    camera_info.D = {-0.3693817071651257, 0.1677750957297015, 0.0007676172676998043, -0.001200264930281811, 0};
    //    camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    camera_info.R = {0.9997156212398773, 0.02379292338064179, 0.001604196362382244,
                     -0.02378757584963585, 0.9997116745775861, -0.003273980524687744,
                     -0.001681631399562056, 0.003234889531517614, 0.9999933537806914};
    camera_info.P = {193.4488673170594, 0, 137.1049880981445, 0,
                     0, 193.4488673170594, 108.951057434082, 0,
                     0, 0, 1, 0};
    cam0.fromCameraInfo(camera_info);

    camera_info.R = {0.9999365173339012, 0.007076042854404519, 0.008768746756027635,
                     -0.007104545173989656, 0.999969566560146, 0.003223568113795293,
                     -0.008745669786783357, -0.003285661430544528, 0.9999563578921555};
    camera_info.K = {198.1315372343827, 0, 132.4194623418875,
                     0, 198.0677328525099, 111.1773834719834,
                     0, 0, 1};
    camera_info.D = {-0.3425648318682812, 0.1238467273033616, 0.0004063467878750188, 0.0004690582572504908, 0};
    cam1.fromCameraInfo(camera_info);

    // Extrinsics
    mat4_1_0 << 1, 0, 0, -0.15,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    mat4_hand_eye = Eigen::Matrix4d::Identity(4,4);

    //    LOG(INFO)<<"extrinsic: "<<mat4_1_0;
    //    LOG(INFO)<<"hand eye: "<<mat4_hand_eye;
}


void get_camera_calib_hkust(image_geometry::PinholeCameraModel& cam0,
                            image_geometry::PinholeCameraModel& cam1,
                            Eigen::Matrix4d& mat4_1_0,
                            Eigen::Matrix4d& mat4_hand_eye,
                            std::string calib_path)
{
    sensor_msgs::CameraInfo camera_info;
    camera_info.height = 260;
    camera_info.width = 346;
    camera_info.K = {263.796, 0, 176.994,
                     0, 263.738, 124.373,
                     0, 0, 1};
    camera_info.distortion_model = "plumb_bob";
    camera_info.D = {-0.386589, 0.157241, 0.000322143, 6.13759e-06};
    camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    //    camera_info.R = {0.999809, 0.0161928, 0.0109163,
    //                     -0.0162088, 0.999868, 0.0013701,
    //                     -0.0108927, -0.00154678, 0.999939};
    camera_info.P = {189.705, 0, 165.382, 0,
                     0, 189.705, 121.295, 0,
                     0, 0, 1, 0};
    cam0.fromCameraInfo(camera_info);

    //    camera_info.R = {0.9993960957463914, 0.0034732142808621717, -0.03457427641222047,
    //                     -0.0035085878889783376, 0.9999933816804096, -0.0009625000798637905,
    //                     0.03457070461958685, 0.0010832257094615543, 0.9994016675011942};
    camera_info.K = {263.485, 0, 162.942,
                     0, 263.276, 118.029,
                     -0.0151344, 0.00133093, 0.999885};
    camera_info.D = {-0.383425, 0.152823, -0.000257745, 0.000268432};
    cam1.fromCameraInfo(camera_info);

    // Extrinsics
    mat4_1_0 << 9.99990798e-01, -6.32492385e-04, -4.24307214e-03, -7.30597639e-02,
            6.44736387e-04,  9.99995631e-01,  2.88489843e-03, -1.23275257e-03,
            4.24122892e-03, -2.88760755e-03,  9.99986837e-01, -1.10420407e-03,
            0, 0, 0, 1.0;
    mat4_hand_eye = Eigen::Matrix4d::Identity(4,4);

    //    LOG(INFO)<<"extrinsic: "<<mat4_1_0;
    //    LOG(INFO)<<"hand eye: "<<mat4_hand_eye;
}

void get_camera_calib_evimo2(image_geometry::PinholeCameraModel& cam0,
                             image_geometry::PinholeCameraModel& cam1,
                             image_geometry::PinholeCameraModel& cam2,
                             Eigen::Matrix4d& mat4_1_0,
                             Eigen::Matrix4d& mat4_2_0,
                             Eigen::Matrix4d& mat4_hand_eye,
                             std::string calib_path)
{
    sensor_msgs::CameraInfo camera_info;
    camera_info.height = 480;
    camera_info.width = 640;

    camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    //samsung mono dvs
    camera_info.K = {519.638, 0, 321.661,
                     0, 519.384, 240.727,
                     0, 0, 1};
    camera_info.distortion_model = "plumb_bob";
    camera_info.D = {0.108306, -0.154485, 0.00103538, -0.000401824};

    cv::Mat K(3, 3, CV_64F, camera_info.K.data());
    cv::Mat P = cv::getOptimalNewCameraMatrix(K, camera_info.D, cv::Size(camera_info.width, camera_info.height), 0);
    for (int r = 0; r<3; r++){
        for (int c=0; c<4; c++){
            camera_info.P[r*4+c]=P.at<double>(r, c);
        }
    }

    cam0.fromCameraInfo(camera_info);

    //prophesee_left
    camera_info.K = {558.417, 0, 324.905,
                     0, 557.475, 225.3,
                     0, 0, 1};
    camera_info.D = {-0.115993, 0.204851, -0.00217161, 0.000676025};

    cam1.fromCameraInfo(camera_info);

    //prophesee_right
    camera_info.K = {556.184, 0, 326.875,
                     0, 555.632, 202.887,
                     0, 0, 1};
    camera_info.D = {-0.110194, 0.205049, 0.00206719, -0.00040706};
    cam2.fromCameraInfo(camera_info);

    // Extrinsics

    Eigen::Matrix4d T_B_0, T_B_1, T_B_2;
    std::vector<double> ext0, ext1, ext2;
    // x, y, z, roll, pitch, yaw
    //samsung
    ext0 = {0.135419,	-0.0214639,	-0.0715952,	-0.00748326,	0.0496968,	-1.79144};
    //prophesee_left
    ext1 = {0.118804,	0.0850843,	-0.0194297,	0.018838,	0.00459314,	-0.195708};
    // prophesee_right
    ext2 = {0.0754507,	-0.119035,	-0.0336873,	-0.0122178,	-0.00473387, 2.93835};

//    Eigen::Matrix4d T1;
//    T1 <<  0.0,   -1.0,   0.0,  0.00,
//            1.0,    0.0,   0.0,  0.00,
//            0.0,    0.0,   1.0,  0.00,
//            0,      0,     0,     1;

//    Eigen::Matrix4d T2;
//    T2 <<  0.0,    0.0,  -1.0,  0.00,
//            0.0,    1.0,   0.0,  0.00,
//            1.0,    0.0,   0.0,  0.00,
//            0,      0,     0,     1;

    {
        tf::Transform E_;
        tf::Quaternion q_;
        tf::Vector3 T_(ext0[0], ext0[1], ext0[2]);
        q_.setRPY(ext0[3], ext0[4], ext0[5]);
        E_.setRotation(q_);
        E_.setOrigin(T_);

        tf::Point p = E_.getOrigin();
        tf::Quaternion q = E_.getRotation();
        tf::Matrix3x3 R1(q);
        Eigen::Matrix3d R2;
        tf::matrixTFToEigen(R1, R2);
        T_B_0.block(0, 0, 3, 3) << R2;
        T_B_0.block(0, 3, 3, 1) << p.x(), p.y(), p.z();
        T_B_0.row(3) << 0, 0, 0, 1;
    }

    {
        tf::Transform E_;
        tf::Quaternion q_;
        tf::Vector3 T_(ext1[0], ext1[1], ext1[2]);
        q_.setRPY(ext1[3], ext1[4], ext1[5]);
        E_.setRotation(q_);
        E_.setOrigin(T_);

        tf::Point p = E_.getOrigin();
        tf::Quaternion q = E_.getRotation();
        tf::Matrix3x3 R1(q);
        Eigen::Matrix3d R2;
        tf::matrixTFToEigen(R1, R2);
        T_B_1.block(0, 0, 3, 3) << R2;
        T_B_1.block(0, 3, 3, 1) << p.x(), p.y(), p.z();
        T_B_1.row(3) << 0, 0, 0, 1;
    }

    {
        tf::Transform E_;
        tf::Quaternion q_;
        tf::Vector3 T_(ext2[0], ext2[1], ext2[2]);
        q_.setRPY(ext2[3], ext2[4], ext2[5]);
        E_.setRotation(q_);
        E_.setOrigin(T_);

        tf::Point p = E_.getOrigin();
        tf::Quaternion q = E_.getRotation();
        tf::Matrix3x3 R1(q);
        Eigen::Matrix3d R2;
        tf::matrixTFToEigen(R1, R2);
        T_B_2.block(0, 0, 3, 3) << R2;
        T_B_2.block(0, 3, 3, 1) << p.x(), p.y(), p.z();
        T_B_2.row(3) << 0, 0, 0, 1;
    }

    mat4_1_0 = T_B_1.inverse()*T_B_0;
    mat4_2_0 = T_B_2.inverse()*T_B_0;
    mat4_hand_eye = T_B_0;
    LOG(INFO)<<"extrinsic: "<<mat4_1_0;
    LOG(INFO)<<"hand eye: "<<mat4_hand_eye;
}


//Load calib params from yaml when cam extrinsics are provided wrt base B (hand)
void get_camera_calib_yaml_mvsec(image_geometry::PinholeCameraModel& cam0,
                                 image_geometry::PinholeCameraModel& cam1,
                                 Eigen::Matrix4d& mat4_1_0,
                                 Eigen::Matrix4d& mat4_hand_eye,
                                 std::string calib_path)
{
    std::vector<sensor_msgs::CameraInfo> camera_info(2);

    YAML::Node calibInfo = YAML::LoadFile(calib_path);

    //Intrinsics
    for (int i=0; i<=1; i++){
        YAML::Node camera = calibInfo["cam"+std::to_string(i)];
        std::vector<int> res = camera["resolution"].as<std::vector<int>>();
        camera_info[i].height = res[1];
        camera_info[i].width = res[0];
        std::vector<double> intrinsics = camera["intrinsics"].as<std::vector<double>>();
        camera_info[i].K = {intrinsics[0], 0., intrinsics[2],
                            0., intrinsics[1], intrinsics[3],
                            0., 0., 1.};

        std::string distortionType = camera["distortion_model"].as<std::string>();
        if (distortionType == "none"){
            camera_info[i].distortion_model = "plumb_bob";
            camera_info[i].D = {0., 0., 0., 0., 0.}; // plumb_bob (rad-tan)
        }
        else if(distortionType == "equidistant"){
            camera_info[i].distortion_model="fisheye";
            camera_info[i].D = camera["distortion_coeffs"].as<std::vector<double>>();
        }
        else if(distortionType == "radtan"){
            camera_info[i].distortion_model="plumb_bob";
            camera_info[i].D = camera["distortion_coeffs"].as<std::vector<double>>();
        }
        // Load rectification matrix form YAML
        //                const YAML::Node R = camera["rectification_matrix"];
        //                for (int r = 0; r<3; r++){
        //                    std::vector<double> tmp = R[r].as<std::vector<double>>();
        //                    for (int c=0; c<3; c++){
        //                        camera_info[i].R[r*3+c]=tmp[c];
        //                    }
        //                }

        //Ignore rectification rotation, work on unrectified images
        camera_info[i].R = {1, 0, 0,
                            0, 1, 0,
                            0, 0, 1};

        // Load projection matrix form YAML
        if (const YAML::Node P = camera["projection_matrix"]){
            for (int r = 0; r<3; r++){
                std::vector<double> tmp = P[r].as<std::vector<double>>();
                for (int c=0; c<4; c++){
                    camera_info[i].P[r*4+c]=tmp[c];
                }
            }
        }
        else {
            cv::Mat K_temp(3, 3, CV_64F, camera_info[i].K.data());
            cv::Mat P_temp = cv::getOptimalNewCameraMatrix(K_temp, camera_info[i].D, cv::Size(camera_info[i].width, camera_info[i].height), 0);

            for (int r = 0; r<3; r++){
                for (int c=0; c<4; c++){
                    camera_info[i].P[r*4+c]=P_temp.at<double>(r, c);
                }
            }
        }

        // Set projection matrix using raw intrinsics
        //                camera_info[i].P = {intrinsics[0], 0., intrinsics[2], 0.,
        //                                    0., intrinsics[1], intrinsics[3], 0.,
        //                                    0., 0., 1., 0.};
    }

    cam0.fromCameraInfo(camera_info[0]);
    camera_info[1].P = camera_info[0].P;
    cam1.fromCameraInfo(camera_info[1]);

    // Extrinsics
    const YAML::Node ext = calibInfo["cam1"]["T_cn_cnm1"];
    for (int r=0; r<4; r++){
        std::vector<double> tmp = ext[r].as<std::vector<double>>();
        mat4_1_0.row(r) = Eigen::Map<Eigen::Matrix<double, 4, 1>>(tmp.data());
    }
    mat4_hand_eye = Eigen::Matrix4d::Identity(4,4);
    LOG(INFO)<<"extrinsic: "<<mat4_1_0;
    LOG(INFO)<<"hand eye: "<<mat4_hand_eye;
}


void get_camera_calib_ESIM(image_geometry::PinholeCameraModel& cam0, 
                           image_geometry::PinholeCameraModel& cam1,
                           Eigen::Matrix4d& mat4_1_0,
                           Eigen::Matrix4d& mat4_hand_eye)
{
    sensor_msgs::CameraInfo camera_info;
    camera_info.height = 180;
    camera_info.width = 240;
    camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    camera_info.distortion_model = "plumb_bob";
    camera_info.K = {200, 0., 120,
                     0., 200, 90,
                     0., 0., 1.};
    camera_info.D = {0., 0., 0., 0., 0.}; // plumb_bob (rad-tan)
    //camera_info.D = {0., 0., 0., 0.}; // fisheye (equidistant) model
    camera_info.P = {200,  0., 120, 0,
                     0., 200,  90, 0,
                     0.,  0.,  1., 0};
    cam0.fromCameraInfo(camera_info);
    cam1.fromCameraInfo(camera_info);

    // Extrinsics
    mat4_1_0 << 1, 0, 0, -0.2,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    // Hand-eye calibration
    mat4_hand_eye << Eigen::Matrix4d::Identity(4,4);

    //   Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    //   std::cout << mat4_hand_eye.format(CleanFmt);
}




// rpg_DAVIS_stereo dataset (Joey ECCV'18)
void get_camera_calib_ECCV18(image_geometry::PinholeCameraModel& cam0, 
                             image_geometry::PinholeCameraModel& cam1,
                             Eigen::Matrix4d& mat4_1_0,
                             Eigen::Matrix4d& mat4_hand_eye)
{
    /*
cam0:
  cam_overlaps: [1]
  camera_model: pinhole
  distortion_coeffs: [-0.3367326394292646, 0.11178850939644308, -0.0014005281258491276,
    -0.00045959441440687044]
  distortion_model: radtan
  intrinsics: [196.63936292910697, 196.7329768429481, 105.06412666477927, 72.47170071387173]
  resolution: [240, 180]
  rostopic: /davis_left/image_raw
cam1:
  T_cn_cnm1:
  - [0.9991089760393723, -0.04098010198963204, 0.010093821797214667, -0.1479883582369969]
  - [0.04098846609277917, 0.9991594254283246, -0.000623077121092687, -0.003289908601915284]
  - [-0.010059803423311134, 0.0010362522169301642, 0.9999488619606629, 0.0026798262366239016]
  - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: [0]
  camera_model: pinhole
  distortion_coeffs: [-0.3462937629552321, 0.12772002965572962, -0.00027205054024332645,
    -0.00019580078540073353]
  distortion_model: radtan
  intrinsics: [196.42564072599785, 196.56440793223533, 110.74517642512458, 88.11310058123058]
  resolution: [240, 180]
  rostopic: /davis_right/image_raw
*/

    sensor_msgs::CameraInfo camera_info;
    camera_info.height = 180;
    camera_info.width = 240;
    camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    camera_info.distortion_model = "plumb_bob";

    // cam0
    camera_info.K = {196.63936292910697, 0., 105.06412666477927,
                     0., 196.7329768429481, 72.47170071387173,
                     0., 0., 1.};
    camera_info.D = {-0.3367326394292646, 0.11178850939644308, -0.0014005281258491276,-0.00045959441440687044, 0.};
    // focal length in P is used in rectify points (to convert undistorted, normalized coordiantes into pixel coordiantes)
    // Use the same P in both cameras, to have the same DSI shape (FOV)
    camera_info.P = {156.925, 0, 108.167, 0,
                     0, 156.925, 78.4205, 0,
                     0, 0, 1, 0};
    cam0.fromCameraInfo(camera_info);

    // cam1
    camera_info.K = {196.42564072599785, 0., 110.74517642512458,
                     0., 196.56440793223533, 88.11310058123058,
                     0., 0., 1.};
    camera_info.D = {-0.3462937629552321, 0.12772002965572962, -0.00027205054024332645, -0.00019580078540073353, 0.};
    cam1.fromCameraInfo(camera_info);

    // Extrinsics
    mat4_1_0 << 0.9991089760393723, -0.04098010198963204, 0.010093821797214667, -0.1479883582369969,
            0.04098846609277917, 0.9991594254283246, -0.000623077121092687, -0.003289908601915284,
            -0.010059803423311134, 0.0010362522169301642, 0.9999488619606629, 0.0026798262366239016,
            0., 0., 0., 1.;

    // Hand-eye calibration
    mat4_hand_eye << 5.363262328777285e-01,    -1.748374625145743e-02,    -8.438296573030597e-01,    -7.009849865398374e-02,
            8.433577587813513e-01,    -2.821937531845164e-02,     5.366109927684415e-01,     1.881333563905305e-02,
            -3.319431623758162e-02,    -9.994488408486204e-01,    -3.897382049768972e-04,    -6.966829200678797e-02,
            0., 0., 0., 1.;
}



// Samsung DVS Gen3 stereo
void get_camera_calib_DVS_Gen3(image_geometry::PinholeCameraModel& cam0, 
                               image_geometry::PinholeCameraModel& cam1,
                               Eigen::Matrix4d& mat4_1_0,
                               Eigen::Matrix4d& mat4_hand_eye)
{
    sensor_msgs::CameraInfo camera_info;
    camera_info.height = 480;
    camera_info.width = 640;
    camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    camera_info.distortion_model = "fisheye";

    // cam0
    camera_info.K = {312.792763, 0.000000, 332.917834,
                     0., 312.783965, 243.939008,
                     0., 0., 1.};
    camera_info.D = {-0.0725278887080172, -0.016272832786070585, 0.018086976118303524, -0.006273794980217994};
    // focal length in P is used in rectify points (to convert undistorted, normalized coordiantes into pixel coordiantes)
    // Use the same P in both cameras, to have the same DSI shape (FOV)
    camera_info.P = {229.308843, 0., 360.397785, 0.,
                     0., 229.308843, 240.487692, 0.,
                     0., 0., 1., 0.};
    cam0.fromCameraInfo(camera_info);

    // cam1
    camera_info.K = {313.830823, 0., 315.546105,
                     0., 313.574021, 236.394256,
                     0., 0., 1.};
    camera_info.D = {-0.08882686690699892, 0.01577827485517159, -0.0052555366228499815, -0.0013447832389448702};
    cam1.fromCameraInfo(camera_info);

    // Extrinsics
    mat4_1_0 << 0.9998198591825752, -0.007121797657941711, 0.017593441455644072, 0.09996202759173385,
            0.00713950571971245, 0.9999740679095885, -0.0009439101790861793, -0.0002694072525916161,
            -0.017586262883626885, 0.001069348618236941, 0.999844777878706, -0.0011054303261930172,
            0., 0., 0., 1.;

    // Hand-eye calibration
    //mat4_hand_eye << Eigen::Matrix4d::Identity(4,4); // Unknown, yet

    mat4_hand_eye << -1., 0, 0, 0,
            0, -1., 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;  // Mounted upside-down, simulate rotation around Z axis
}

