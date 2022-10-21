/*
* \file utils.cpp
* \brief utility functions like saving depth and confidence maps
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

#include <mapper_emvs_stereo/utils.hpp>

#include <sstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

void saveDepthMaps(const cv::Mat& depth_map, 
                   const cv::Mat& confidence_map,
                   const cv::Mat& semidense_mask,
                   const cv::Mat& depth_map_dense,
                   const float min_depth,
                   const float max_depth,
                   const std::string& suffix,
                   const std::string& out_path)
{
    // save masked depth points with double precision
    std::ofstream of;
    std::string savePath(out_path + "depth_points_" + suffix);
    savePath.append(".txt");
    of.open(savePath, std::ofstream::out);
    if(of.is_open())
    {
        for (int r=0; r<depth_map.rows; r++) {
            for (int c=0; c<depth_map.cols; c++) {
                if(semidense_mask.at<uint8_t>(r, c)>0){
                    of << c << " " << r << " " << depth_map.at<float>(r, c) << "\n";
                }
            }
        }
    }
    of.close();

    // Save depth map, confidence map and semi-dense mask
    std::stringstream ss_suffix;
    ss_suffix << suffix << ".png";
    // Save semi-dense mask as an image
    //cv::imwrite(out_path + "semidense_mask_" + ss_suffix.str(), 255 * semidense_mask);

    // Save confidence map as an 8-bit image
    cv::Mat confidence_map_255;
    cv::normalize(confidence_map, confidence_map_255, 0, 255.0, cv::NORM_MINMAX, CV_32FC1);
//    cv::imwrite(out_path + "confidence_map_" + ss_suffix.str(), confidence_map_255);
    cv::imwrite(out_path + "confidence_map_negated_" + ss_suffix.str(), 255-confidence_map_255);

    // Normalize depth map using given min and max depth values
    cv::Mat depth_map_255 = (depth_map - min_depth) * (255.0 / (max_depth - min_depth));
//    cv::imwrite(out_path + "depth_map_" + ss_suffix.str(), depth_map_255);

    // Save pseudo-colored depth map on white canvas
    //    cv::Mat depthmap_8bit, depthmap_color;
    //    depth_map_255.convertTo(depthmap_8bit, CV_8U);
    //    cv::applyColorMap(depthmap_8bit, depthmap_color, cv::COLORMAP_RAINBOW);
    //    cv::Mat depth_on_canvas = cv::Mat(depth_map.rows, depth_map.cols, CV_8UC3, cv::Scalar(1,1,1)*255);
    //    depthmap_color.copyTo(depth_on_canvas, semidense_mask);
    //    cv::imwrite(out_path + "depth_colored_" + ss_suffix.str(), depth_on_canvas);

    // Save dilated colored depth map on black canvas
    //  cv::applyColorMap(depthmap_8bit, depthmap_color, cv::COLORMAP_JET);
    //  depth_on_canvas = cv::Mat(depth_map.rows, depth_map.cols, CV_8UC3, cv::Scalar(1,1,1)*0);
    //  depthmap_color.copyTo(depth_on_canvas, semidense_mask);
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(3,3));
    //  cv::dilate(depth_on_canvas, depth_on_canvas, element);
    //  cv::imwrite(out_path + "depth_colored_dilated" + ss_suffix.str(), depth_on_canvas);


    // Jet colormap with black background for inverse depth maps (similar to ESVO)
    cv::Mat invdepthmap_8bit, invdepthmap_color;
    cv::Mat invmap = 1./depth_map;
    float mod_max_depth = 1 * max_depth;
    cv::Mat invdepth_map_255 = (invmap - 1./mod_max_depth)  / (1./min_depth - 1./mod_max_depth) * 255.;
    invdepth_map_255.convertTo(invdepthmap_8bit, CV_8U);
    cv::applyColorMap(invdepthmap_8bit, invdepthmap_color, cv::COLORMAP_JET);
    cv::Mat invdepth_on_canvas = cv::Mat(depth_map.rows, depth_map.cols, CV_8UC3, cv::Scalar(1,1,1)*0);
    invdepthmap_color.copyTo(invdepth_on_canvas, semidense_mask);
//    cv::imwrite(out_path + "inv_depth_colored_" + ss_suffix.str(), invdepth_on_canvas);
    element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::dilate(invdepth_on_canvas, invdepth_on_canvas, element);
    cv::imwrite(out_path + "inv_depth_colored_dilated_" + ss_suffix.str(), invdepth_on_canvas);

    //    if (depth_map_dense.rows > 0)
    //    {
    //        // Normalize depth map using given min and max depth values
    //        depth_map_255 = (depth_map_dense - min_depth) * (255.0 / (max_depth - min_depth));
    //        // Save pseudo-colored depth map on white canvas
    //        depth_map_255.convertTo(depthmap_8bit, CV_8U);
    //        cv::applyColorMap(depthmap_8bit, depthmap_color, cv::COLORMAP_RAINBOW);
    //        cv::imwrite(out_path + "depth_colored_dense_" + ss_suffix.str(), depthmap_color);
    //    }
}


void saveDepthMaps(const cv::Mat& depth_map,
                   const cv::Mat& confidence_map,
                   const cv::Mat& semidense_mask,
                   const float min_depth,
                   const float max_depth,
                   const std::string& suffix,
                   const std::string& out_path)
{
    cv::Mat depth_map_dense;
    saveDepthMaps(depth_map, confidence_map, semidense_mask, depth_map_dense, min_depth, max_depth, suffix, out_path);
}


// Harmonic mean of two DSIs in mapper objects
void fuseDSIs_HarmonicMean(const EMVS::MapperEMVS& mapper0,
                           const EMVS::MapperEMVS& mapper1,
                           EMVS::MapperEMVS& mapper_fused)
{
    mapper_fused.dsi_.resetGrid();
    mapper_fused.dsi_.addTwoGrids(mapper0.dsi_); // Initialize DSI
    mapper_fused.dsi_.harmonicMeanTwoGrids(mapper1.dsi_);
}

// Sum (e.g., arithmetic mean) of two DSIs in mapper objects
void fuseDSIs_Sum(const EMVS::MapperEMVS& mapper0,
                  const EMVS::MapperEMVS& mapper1,
                  EMVS::MapperEMVS& mapper_fused)
{
    mapper_fused.dsi_.resetGrid();
    mapper_fused.dsi_.addTwoGrids(mapper0.dsi_); // Initialize DSI
    mapper_fused.dsi_.addTwoGrids(mapper1.dsi_);
}


// Temporary
double computeFocus_MeanSquare(const cv::Mat& img)
{
    return cv::mean(img.mul(img))[0];
}

double computeFocus_Variance(const cv::Mat& img)
{
    cv::Scalar f_mean, f_stddev;
    cv::meanStdDev(img, f_mean, f_stddev);
    return f_stddev[0]*f_stddev[0]; // variance
}


// Fusion: Harmonic mean of the local focus scores of both DSIs
void fuseDSIs_HarmonicMeanOfLocalFocus(const EMVS::MapperEMVS& mapper0,
                                       const EMVS::MapperEMVS& mapper1,
                                       const image_geometry::PinholeCameraModel& cam0,
                                       const image_geometry::PinholeCameraModel& cam1,
                                       const EMVS::ShapeDSI& dsi_shape,
                                       const int focus_method, // 0 for variance, 1 for MS
                                       EMVS::MapperEMVS& mapper_focus_fused)
{
    // Local focus score of each DSI
    // Using Gaussian weights (by convolution)
    EMVS::MapperEMVS mapper_focus0(cam0, dsi_shape);
    EMVS::MapperEMVS mapper_focus1(cam1, dsi_shape);

    mapper_focus0.dsi_.resetGrid();
    mapper_focus0.dsi_.addTwoGrids(mapper0.dsi_); // Initialize DSI
    mapper_focus0.dsi_.computeLocalFocusInPlace(focus_method);

    mapper_focus1.dsi_.resetGrid();
    mapper_focus1.dsi_.addTwoGrids(mapper1.dsi_); // Initialize DSI
    mapper_focus1.dsi_.computeLocalFocusInPlace(focus_method);

    // Harmonic mean of the local focus scores of both DSIs
    mapper_focus_fused.dsi_.resetGrid();
    mapper_focus_fused.dsi_.addTwoGrids(mapper_focus0.dsi_); // Initialize DSI
    mapper_focus_fused.dsi_.harmonicMeanTwoGrids(mapper_focus1.dsi_);
}


void accumulateEvents(const std::vector<dvs_msgs::Event>& events,
                      const bool use_polarity,
                      cv::Mat& img)
{
    if (use_polarity)
    {
        cv::Mat imgf = cv::Mat::zeros(img.size(),CV_32FC1);
        for(auto e: events)
        {
            imgf.at<float>(e.y,e.x) += (e.polarity ? 1 : -1);
        }
        // Normalize to [0,255], with value 128 corresponding to zero events
        double min, max;
        cv::minMaxLoc(imgf, &min, &max);
        const double half_range = std::max( fabs(min), fabs(max) );
        if (half_range > 0)
        {
            imgf = (imgf * (128/half_range)) + 128;
            imgf.convertTo(img,CV_8UC1);
        } else
        {
            img.setTo(128);
        }
    } else
    {
        img.setTo(0);
        for(auto e: events)
        {
            img.at<uchar>(e.y,e.x) += 1;
        }
        cv::normalize(img, img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    }
}
