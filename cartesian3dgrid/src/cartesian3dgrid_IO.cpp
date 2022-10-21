/*
* \file cartesian3dgrid_IO.cpp
* \brief functions for saving DSI
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

#include "cartesian3dgrid/cartesian3dgrid.h"

#include <iostream>
#include <string.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <opencv2/highgui.hpp>

#include <cnpy.h>


//--------------------- Output for Numpy -----------------------------------------

int Grid3D::writeGridNpy(const char filename[]) const
{
  cnpy::npy_save(std::string(filename),
                 &data_array_[0],
                 {size_[2], size_[1], size_[0]}, "w");
  return 0;
}


void Grid3D::imwriteSlices(const char prefix[], const unsigned int dimIdx, bool normalize_by_minmax) const
{
  // Option: write images normalizing by the maximum value over grid
  float min, max, range;
  if (normalize_by_minmax)
  {
    auto it = std::minmax_element(std::begin(data_array_), std::end(data_array_));
    min = *it.first;
    max = *it.second;
    range = max - min;
    //std::cout << "Min value " << min << std::endl;
    //std::cout << "Max value " << max << std::endl;
  }
  
  #pragma omp parallel for
  for (int idx_slice = 0; idx_slice < size_[dimIdx]; idx_slice++)
  {
    // Get slice
    cv::Mat slice_img = getSlice(idx_slice, dimIdx);
    
    // Write slice to disk
    std::stringstream ss;
    ss << std::string(prefix) << std::setfill('0') << std::setw(3) << idx_slice << ".png";
    cv::Mat slice_u;
    if (normalize_by_minmax)
    {
      // All slices are normalized using the range
      slice_img = (slice_img - min) / range;
      slice_img.convertTo(slice_u,CV_8UC1,255);
    }
    else
    {
      // Each slice is normalized independently from the rest
      cv::normalize(slice_img, slice_u, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    }
    imwrite(ss.str(),slice_u);
  }
}
