/*
* \file cartesian3dgrid.cpp
* \brief utility functions for handling 3D volumes
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

#include "../include/cartesian3dgrid/cartesian3dgrid.h"

#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


Grid3D::Grid3D()
{
  deallocate();
}


Grid3D::Grid3D(const unsigned int dimX, const unsigned int dimY, const unsigned int dimZ)
{
  deallocate();
  allocate(dimX,dimY,dimZ);
}


void Grid3D::allocate(const unsigned int dimX, const unsigned int dimY, const unsigned int dimZ)
{
  size_[0] = dimX;
  size_[1] = dimY;
  size_[2] = dimZ;
  numCells_ = size_[0]*size_[1]*size_[2];
  data_array_.resize(numCells_);
  resetGrid();
}


Grid3D::~Grid3D()
{
  deallocate();
}


void Grid3D::deallocate()
{
  size_[0] = size_[1] = size_[2] = 0;
  data_array_.clear();
}


void Grid3D::printInfo() const
{
  std::cout << "Grid3D Dimensions: " << "(" << size_[0] << "," << size_[1] << "," << size_[2] << ")" << std::endl;
  std::cout << "Grid3D Data_array size: " << data_array_.size() << std::endl;
}

void Grid3D::resetGrid()
{
  std::fill(data_array_.begin(), data_array_.end(), 0.f);
}

cv::Mat Grid3D::getSlice(const unsigned int sliceIdx, const unsigned int dimIdx) const
{
  cv::Mat slice;
  
  // simplest way
  if (dimIdx==0)
  {
    // X-slice
    unsigned int u_size = size_[2], // Z acts as X-axis
                 v_size = size_[1];
    slice = cv::Mat(v_size,u_size,CV_32FC1);
    for(unsigned int u=0; u<u_size; u++)
      for(unsigned int v=0; v<v_size; v++)
        slice.at<float>(v,u) = getGridValueAt(sliceIdx,v,u);
  }
  else if (dimIdx==1)
  {
    // Y-slice
    unsigned int u_size = size_[2], // Z acts as X-axis
                 v_size = size_[0];
    slice = cv::Mat(v_size,u_size,CV_32FC1);
    for(unsigned int u=0; u<u_size; u++)
      for(unsigned int v=0; v<v_size; v++)
        slice.at<float>(v,u) = getGridValueAt(v,sliceIdx,u);
  }
  else if (dimIdx==2)
  {
    // Z-slice
    unsigned int u_size = size_[0], 
                 v_size = size_[1];
    slice = cv::Mat(v_size,u_size,CV_32FC1); // (y,x) as in images
    for(unsigned int v=0; v<v_size; v++)
      for(unsigned int u=0; u<u_size; u++)
        slice.at<float>(v,u) = getGridValueAt(u,v,sliceIdx);
  }
  else
  {
    std::cout << "ERROR. dimIdx should be 0, 1 or 2" << std::endl;
  }
  
  return slice;
}

void Grid3D::collapseMaxZSlice(cv::Mat* max_val, cv::Mat* max_pos_idx) const
{
  // Z-slice
  unsigned int u_size = size_[0], v_size = size_[1], w_size = size_[2];
  *max_val = cv::Mat(v_size,u_size, CV_32FC1); // (y,x) as in images
  *max_pos_idx = cv::Mat(v_size,u_size, CV_8U); // WARNING: Max 256 depth layers
  
  std::vector<float> grid_vals_vec(w_size);
  for(unsigned int v=0; v<v_size; v++)
  {
    for(unsigned int u=0; u<u_size; u++)
    {
      // Build vector containing the grid values at a given (u,v), as a function of depth
      for(unsigned int k=0; k<w_size; ++k)
        grid_vals_vec.at(k) = getGridValueAt(u,v,k);
      
      // Look for maximum of vector and its location
      auto max = std::max_element(std::begin(grid_vals_vec), std::end(grid_vals_vec));
      (*max_val).at<float>(v,u) = *max;
      (*max_pos_idx).at<uchar>(v,u) = std::distance(std::begin(grid_vals_vec), max);
    }
  }
}

void Grid3D::collapseMinZSlice(cv::Mat* max_val, cv::Mat* max_pos_idx) const
{
  // Z-slice
  unsigned int u_size = size_[0], v_size = size_[1], w_size = size_[2];
  *max_val = cv::Mat(v_size,u_size, CV_32FC1); // (y,x) as in images
  *max_pos_idx = cv::Mat(v_size,u_size, CV_8U); // WARNING: Max 256 depth layers
  
  std::vector<float> grid_vals_vec(w_size);
  for(unsigned int v=0; v<v_size; v++)
  {
    for(unsigned int u=0; u<u_size; u++)
    {
      // Build vector containing the grid values at a given (u,v), as a function of depth
      for(unsigned int k=0; k<w_size; ++k)
        grid_vals_vec.at(k) = getGridValueAt(u,v,k);
      
      // Look for maximum of vector and its location
      auto max = std::min_element(std::begin(grid_vals_vec), std::end(grid_vals_vec));
      (*max_val).at<float>(v,u) = *max;
      (*max_pos_idx).at<uchar>(v,u) = std::distance(std::begin(grid_vals_vec), max);
    }
  }
}


double Grid3D::computeMeanSquare() const
{
  double result = 0.;
  for (int i=0; i<numCells_; i++)
  {
    double tmp = (double) data_array_.at(i);
    result += tmp*tmp;
  }
  
  return result/numCells_;
}


void Grid3D::getMinMax(float* min_val, float* max_val, unsigned long* min_pos, unsigned long* max_pos) const
{
  auto result = std::minmax_element(data_array_.begin(), data_array_.end());
  *min_val = *result.first;
  *max_val = *result.second;

  if (min_pos !=NULL)
    *min_pos = (unsigned long) (result.first - data_array_.begin());

  if (max_pos !=NULL)
    *max_pos = (unsigned long) (result.second - data_array_.begin());
}



void Grid3D::collapseZSliceByGradMag(cv::Mat* confidence, cv::Mat* depth_cell_indices, int half_patchsize)
{
  // Z-slice
  unsigned int u_size = size_[0], v_size = size_[1], w_size = size_[2];
  *confidence = cv::Mat::zeros(v_size,u_size, CV_32FC1); // (y,x) as in images
  *depth_cell_indices = cv::Mat::zeros(v_size,u_size, CV_8U); // WARNING: Max 256 depth layers

  // Bell-shaped mask for a patch
  const int patch_size = 2 * half_patchsize + 1;
  cv::Mat gaussian_kernel = cv::getGaussianKernel(patch_size, -1, CV_32F);
  cv::Mat gaussian_kernel_2d = gaussian_kernel * gaussian_kernel.t();
  
  for(unsigned int k=0; k<w_size; ++k)
  {
    // Get depth plane
    float *pgrid = &data_array_[k * u_size * v_size];
    cv::Mat slice_dsi(v_size, u_size, CV_32F, pgrid);
    
    // Compute local gradient magnitude (focus measure) of current depth slice of the DSI
    cv::Mat grad_x, grad_y, grad_mag;
    cv::Sobel( slice_dsi, grad_x, CV_32F, 1, 0);
    cv::Sobel( slice_dsi, grad_y, CV_32F, 0, 1);
    grad_mag = grad_x.mul(grad_x) + grad_y.mul(grad_y);
    
    // Retain maximum local focus per pixel (i.e., per optical ray)
    for(int y=half_patchsize; y<v_size-half_patchsize; ++y)
    {
      for(int x=half_patchsize; x<u_size-half_patchsize; ++x)
      {
        // TODO
        // Compute the focus measure on a small patch around (x,y)
        //float focus = grad_mag.at<float>(y,x);  // Causes double edges
        
        cv::Mat patch = grad_mag(cv::Rect(x-half_patchsize, y-half_patchsize, patch_size, patch_size)); //.mul(gaussian_kernel_2d);
        float focus = cv::mean(patch)[0];

        // Select maximum focus value along the optical ray
        if(focus > confidence->at<float>(y,x))
        {
          confidence->at<float>(y,x) = focus; // max focus along optical ray
          depth_cell_indices->at<uchar>(y,x) = k;
        }
      }
    }
  }
  
  // Return gradient magnitude instead of its square
  cv::sqrt(*confidence, *confidence);
}


void Grid3D::collapseZSliceByLaplacianMag(cv::Mat* confidence, cv::Mat* depth_cell_indices)
{
  // Z-slice
  unsigned int u_size = size_[0], v_size = size_[1], w_size = size_[2];
  *confidence = cv::Mat::zeros(v_size,u_size, CV_32FC1); // (y,x) as in images
  *depth_cell_indices = cv::Mat::zeros(v_size,u_size, CV_8U); // WARNING: Max 256 depth layers
  
  for(unsigned int k=0; k<w_size; ++k)
  {
    // Get depth plane
    float *pgrid = &data_array_[k * u_size * v_size];
    cv::Mat slice_dsi(v_size, u_size, CV_32F, pgrid);
    
    // Compute local focus measure of current depth slice of the DSI
    cv::Mat slice_high_freq, slice_high_freq_mag;
    cv::Laplacian( slice_dsi, slice_high_freq, CV_32FC1, 5);
    slice_high_freq_mag = slice_high_freq.mul(slice_high_freq);
    
    // Retain maximum local focus per pixel (i.e., per optical ray)
    for(int y=0; y<v_size; ++y)
    {
      for(int x=0; x<u_size; ++x)
      {
        // Compute the focus measure on a small patch around (x,y)
        float focus = slice_high_freq_mag.at<float>(y,x);

        // Select maximum focus value along the optical ray
        if(focus > confidence->at<float>(y,x))
        {
          confidence->at<float>(y,x) = focus; // max focus along optical ray
          depth_cell_indices->at<uchar>(y,x) = k;
        }
      }
    }
  }
  
  // Return gradient magnitude instead of its square
  cv::sqrt(*confidence, *confidence);
}


void Grid3D::collapseZSliceByDoG(cv::Mat* confidence, cv::Mat* depth_cell_indices)
{
  // Z-slice
  unsigned int u_size = size_[0], v_size = size_[1], w_size = size_[2];
  *confidence = cv::Mat::zeros(v_size,u_size, CV_32FC1); // (y,x) as in images
  *depth_cell_indices = cv::Mat::zeros(v_size,u_size, CV_8U); // WARNING: Max 256 depth layers
  
  // Compute DoG filtered image
  const double sigma = 0.5; // 1.0
  
  //const double sigma2 = sigma * 3.0; // DoG
  const double sigma2 = sigma * 1.6; // LoG
  
  for(unsigned int k=0; k<w_size; ++k)
  {
    // Get depth plane
    float *pgrid = &data_array_[k * u_size * v_size];
    cv::Mat slice_dsi(v_size, u_size, CV_32F, pgrid);
    
    // Local means with two Gaussian kernel widths
    cv::Mat slice_local_mean1, slice_local_mean2;
    // Compute DoG filtered image
    cv::GaussianBlur(slice_dsi, slice_local_mean1, cv::Size(0,0), sigma,sigma, cv::BORDER_REFLECT);
    cv::GaussianBlur(slice_dsi, slice_local_mean2, cv::Size(0,0), sigma2,sigma2, cv::BORDER_REFLECT);
    slice_local_mean1 -= slice_local_mean2;

    // Retain maximum local focus per pixel (i.e., per optical ray)
    for(int y=0; y<v_size; ++y)
    {
      for(int x=0; x<u_size; ++x)
      {
        // Get the local variance (i.e., focus) at (x,y)
        float focus = fabs(slice_local_mean1.at<float>(y,x));

        // Select maximum focus value along the optical ray
        if(focus > confidence->at<float>(y,x))
        {
          confidence->at<float>(y,x) = focus; // max focus along optical ray
          depth_cell_indices->at<uchar>(y,x) = k;
        }
      }
    }
  }
}


void Grid3D::collapseZSliceByLocalVar(cv::Mat* confidence, cv::Mat* depth_cell_indices)
{
  // Z-slice
  unsigned int u_size = size_[0], v_size = size_[1], w_size = size_[2];
  *confidence = cv::Mat::zeros(v_size,u_size, CV_32FC1); // (y,x) as in images
  *depth_cell_indices = cv::Mat::zeros(v_size,u_size, CV_8U); // WARNING: Max 256 depth layers
  
  const double sigma = 0.5;
  
  for(unsigned int k=0; k<w_size; ++k)
  {
    // Get depth plane
    float *pgrid = &data_array_[k * u_size * v_size];
    cv::Mat slice_dsi(v_size, u_size, CV_32F, pgrid);
    
    // Compute local mean, local MS and then, local variance
    cv::Mat slice_local_mean, slice_local_MS;
    cv::GaussianBlur(slice_dsi, slice_local_mean, cv::Size(0,0), sigma,sigma, cv::BORDER_REFLECT);
    cv::GaussianBlur(slice_dsi.mul(slice_dsi), slice_local_MS, cv::Size(0,0), sigma,sigma, cv::BORDER_REFLECT);
    cv::Mat slice_local_var = slice_local_MS - slice_local_mean.mul(slice_local_mean);
    cv::threshold(slice_local_var, slice_local_var, 0., 0., cv::THRESH_TOZERO); // guarantee non-negativeness
    
    // Retain maximum local focus per pixel (i.e., per optical ray)
    for(int y=0; y<v_size; ++y)
    {
      for(int x=0; x<u_size; ++x)
      {
        // Get the local variance (i.e., focus) at (x,y)
        float focus = fabs(slice_local_var.at<float>(y,x));

        // Select maximum focus value along the optical ray
        if(focus > confidence->at<float>(y,x))
        {
          confidence->at<float>(y,x) = focus; // max focus along optical ray
          depth_cell_indices->at<uchar>(y,x) = k;
        }
      }
    }
  }
  
  // Return std instead of variance
  //cv::sqrt(*confidence, *confidence);
}


void Grid3D::collapseZSliceByLocalMeanSquare(cv::Mat* confidence, cv::Mat* depth_cell_indices)
{
  // Z-slice
  unsigned int u_size = size_[0], v_size = size_[1], w_size = size_[2];
  *confidence = cv::Mat::zeros(v_size,u_size, CV_32FC1); // (y,x) as in images
  *depth_cell_indices = cv::Mat::zeros(v_size,u_size, CV_8U); // WARNING: Max 256 depth layers
  
  const double sigma = 0.5;
  
  for(unsigned int k=0; k<w_size; ++k)
  {
    // Get depth plane
    float *pgrid = &data_array_[k * u_size * v_size];
    cv::Mat slice_dsi(v_size, u_size, CV_32F, pgrid);
    
    // Compute filtered image
    cv::Mat slice_local_MS;
    cv::GaussianBlur(slice_dsi.mul(slice_dsi), slice_local_MS, cv::Size(0,0), sigma,sigma, cv::BORDER_REFLECT);
    
    // Retain maximum local focus per pixel (i.e., per optical ray)
    for(int y=0; y<v_size; ++y)
    {
      for(int x=0; x<u_size; ++x)
      {
        // Get the local MS (i.e., focus) at (x,y)
        float focus = slice_local_MS.at<float>(y,x);

        // Select maximum focus value along the optical ray
        if(focus > confidence->at<float>(y,x))
        {
          confidence->at<float>(y,x) = focus; // max focus along optical ray
          depth_cell_indices->at<uchar>(y,x) = k;
        }
      }
    }
  }
  
  // Return sqrt of MS
  //cv::sqrt(*confidence, *confidence);
}


void Grid3D::computeLocalFocusInPlace(int focus_method)
{
  switch (focus_method)
  {
    case 1:
      computeLocalMeanSquareInPlace();
      break;
    default:
      computeLocalVarInPlace();
      break;
  }
}


void Grid3D::computeLocalVarInPlace()
{
  // Z-slice
  unsigned int u_size = size_[0], v_size = size_[1], w_size = size_[2];
  
  const double sigma = 0.5;
  
  #pragma omp parallel for
  for(unsigned int k=0; k<w_size; ++k)
  {
    // Get depth plane
    float *pgrid = &data_array_[k * u_size * v_size];
    cv::Mat slice_dsi(v_size, u_size, CV_32F, pgrid);
    
    // Compute local variance by convolution, including Gaussian smoothing
    cv::Mat slice_local_mean, slice_local_MS; // Local mean and MS per Z-slice
    cv::GaussianBlur(slice_dsi, slice_local_mean, cv::Size(0,0), sigma,sigma, cv::BORDER_REFLECT);
    cv::GaussianBlur(slice_dsi.mul(slice_dsi), slice_local_MS, cv::Size(0,0), sigma,sigma, cv::BORDER_REFLECT);
    cv::Mat slice_local_var = slice_local_MS - slice_local_mean.mul(slice_local_mean);
    cv::threshold(slice_local_var, slice_local_var, 0., 0., cv::THRESH_TOZERO); // guarantee non-negativeness
    
    //Stddev instead of variance
    cv::sqrt(slice_local_var, slice_local_var);
    
    // Overwrite slice value
    slice_local_var.copyTo(slice_dsi);
  }
}


void Grid3D::computeLocalMeanSquareInPlace()
{
  
  // Z-slice
  unsigned int u_size = size_[0], v_size = size_[1], w_size = size_[2];
  
  const double sigma = 0.5;
  
  #pragma omp parallel for
  for(unsigned int k=0; k<w_size; ++k)
  {
    // Get depth plane
    float *pgrid = &data_array_[k * u_size * v_size];
    cv::Mat slice_dsi(v_size, u_size, CV_32F, pgrid);
    
    // Compute local MS by convolution, including Gaussian smoothing
    cv::Mat slice_local_MS;
    cv::GaussianBlur(slice_dsi.mul(slice_dsi), slice_local_MS, cv::Size(0,0), sigma,sigma, cv::BORDER_REFLECT);
    
    // Overwrite slice value
    slice_local_MS.copyTo(slice_dsi);
  }
}
