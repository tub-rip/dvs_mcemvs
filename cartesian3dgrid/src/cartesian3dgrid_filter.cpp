#include "cartesian3dgrid/cartesian3dgrid.h"

#include <iostream>
#include <vector>

#include "cartesian3dgrid/gaussianiir3d.h"


void Grid3D::smoothInPlaceIIR3D(const float sigma_x, const float sigma_y, const float sigma_z)
{
  int numsteps = 3;
  gaussianiir3d(&(data_array_.at(0)), size_[0], size_[1], size_[2], sigma_x, sigma_y, sigma_z, numsteps);
}


/**
 * \brief Diffuse grid using the heat equation (isotropic Gaussian blur)
 */
void diffuse ( std::vector<float>* grid, const unsigned int width, const unsigned int height, const unsigned int depth, 
               const float sigma)
{
  std::vector<float> grid_new(width*height*depth,0.f); // temporary array
  
  const float dt_CFL = 1.f / 12.f; // 3D heat equation, explicit scheme stability condition
  const float t_final = 0.5f * sigma*sigma;
  const float dt = std::min( 0.5f*dt_CFL, 0.5f*t_final);
  unsigned int steps = std::ceil(t_final / dt);

  const int stride_x = 1, stride_y = width, stride_z = width*height;

  while ( steps-- )
  {
    for (int iz=0, p=0; iz < depth; iz++)
    {
      for (int iy=0; iy < height; iy++)
      {
        for (int ix=0; ix < width; ix++, p++)
        {
          // Boundary conditions: homogeneous Neumann
          int dx_pos =  stride_x; if (ix+1 >= width){dx_pos = 0;}
          int dx_neg = -stride_x; if (ix-1 < 0){dx_neg = 0;}
          int dy_pos =  stride_y; if (iy+1 >= height){dy_pos = 0;}
          int dy_neg = -stride_y; if (iy-1 < 0){dy_neg = 0;}
          int dz_pos =  stride_z; if (iz+1 >= depth){dz_pos = 0;}
          int dz_neg = -stride_z; if (iz-1 < 0){dz_neg = 0;}

          // Index of current point
          //int p = stride_x*ix + stride_y*iy + stride_z*iz;
          // Compute 3D Laplacian: sum of second order derivatives
          float grid_p = grid->at(p);
          float Dxx = grid->at(p+dx_pos) - 2*grid_p + grid->at(p+dx_neg);
          float Dyy = grid->at(p+dy_pos) - 2*grid_p + grid->at(p+dy_neg);
          float Dzz = grid->at(p+dz_pos) - 2*grid_p + grid->at(p+dz_neg);
          // Update current point with explicit Euler scheme.
          // Stability condition (CFL) is 1-6*dt > 0
          grid_new.at(p) = grid_p + dt * ( Dxx + Dyy + Dzz );
        }
      }
    }
    grid->swap(grid_new);
  }
}


//void Grid3D::smoothInPlace(const float sigma_x, const float sigma_y, const float sigma_z)
void Grid3D::smoothInPlace(const float sigma)
{
  diffuse ( &data_array_, size_[0], size_[1], size_[2], sigma);
}


void laplacian3D ( std::vector<float>* grid, const unsigned int width, const unsigned int height, const unsigned int depth)
{
  std::vector<float> grid_new(width*height*depth,0.f); // temporary array
  
  const int stride_x = 1, stride_y = width, stride_z = width*height;

  for (int iz=0, p=0; iz < depth; iz++)
  {
    for (int iy=0; iy < height; iy++)
    {
      for (int ix=0; ix < width; ix++, p++)
      {
        // Boundary conditions: homogeneous Neumann
        int dx_pos =  stride_x; if (ix+1 >= width){dx_pos = 0;}
        int dx_neg = -stride_x; if (ix-1 < 0){dx_neg = 0;}
        int dy_pos =  stride_y; if (iy+1 >= height){dy_pos = 0;}
        int dy_neg = -stride_y; if (iy-1 < 0){dy_neg = 0;}
        int dz_pos =  stride_z; if (iz+1 >= depth){dz_pos = 0;}
        int dz_neg = -stride_z; if (iz-1 < 0){dz_neg = 0;}
        
        // Index of current point
        //int p = stride_x*ix + stride_y*iy + stride_z*iz;
        // Compute 3D Laplacian: sum of second order derivatives
        float grid_p = grid->at(p);
        float Dxx = grid->at(p+dx_pos) - 2*grid_p + grid->at(p+dx_neg);
        float Dyy = grid->at(p+dy_pos) - 2*grid_p + grid->at(p+dy_neg);
        float Dzz = grid->at(p+dz_pos) - 2*grid_p + grid->at(p+dz_neg);
        grid_new.at(p) = Dxx + Dyy + Dzz;
      }
    }
  }
  grid->swap(grid_new);
}


void Grid3D::laplacianInPlace()
{
  laplacian3D ( &data_array_, size_[0], size_[1], size_[2]);
}


double Grid3D::computeMoranIndexGaussianWeights(float sigma) const
{
  if (sigma < 0.2)
  {
    // Need some "minimum" width to provide meaningful results
    sigma = 0.2;
  }
  
  // Standardized grid values
  double mean, stddev;
  computeMeanStd(&mean, &stddev);
  std::cout << "Grid3D: Mean = " << mean << "  std = " << stddev << std::endl;
  
  float inv_stddev_f = 1.f / stddev;
  std::vector<float> data_array_standardized(numCells_,0.f);
  for (int i=0; i<numCells_; i++)
    data_array_standardized.at(i) = (data_array_.at(i) - mean) * inv_stddev_f;
    
  // Filter the grid values
  std::vector<float> data_array_standardized_smooth;
  data_array_standardized_smooth = data_array_standardized; // copy
  //diffuse(&data_array_standardized_smooth, size_[0], size_[1], size_[2], sigma);
  gaussianiir3d(&(data_array_standardized_smooth.at(0)), size_[0], size_[1], size_[2], sigma, sigma, sigma, 3);
  
  // Compute central weight of Gaussian kernel and its complement, the sum of weights of the neighbors
  const int half_size = 2 * std::ceil(3.f*sigma); // 2 to mitigate boundary issues
  const int kernel_size = 2*half_size + 1;
  std::vector<float> weights(kernel_size*kernel_size*kernel_size, 0.f);
  unsigned int idx_center = half_size + kernel_size*(half_size + kernel_size*half_size);
  weights.at(idx_center) = 1.f;
  //diffuse(&weights, kernel_size, kernel_size, kernel_size, sigma);
  gaussianiir3d(&(weights.at(0)), kernel_size, kernel_size, kernel_size, sigma, sigma, sigma, 3);
  
  const float central_weight = weights.at(idx_center);
  /*
  //DEBUG
  std::cout << "sigma = " << sigma << std::endl;
  std::cout << "kernel_size = " << kernel_size << std::endl;
  std::cout << "central_weight = " << central_weight << std::endl;
  // print kernel
  std::cout << "weights" << std::endl;
  for (int iz=0; iz < kernel_size; iz++)
  {
    std::cout << "iz = " << iz << std::endl;
    for (int iy=0; iy < kernel_size; iy++)
    {
      for (int ix=0; ix < kernel_size; ix++)
      {
        int idx = ix + kernel_size*(iy + kernel_size*iz);
        std::cout << weights.at(idx) << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
  */
  
  // Compute sum of the weights (of the neighbors)
  const float sum_weights = 1.f - central_weight;
  /*
  //DEBUG
  weights.at(idx_center) = 0.f; // "remove" central point
  float sum_weights2 = 0.f;
  const int numCells = kernel_size*kernel_size*kernel_size;
  for (int i=0; i<numCells; i++)
    sum_weights2 += weights.at(i);
  
  std::cout << "sum_weights = " << sum_weights << std::endl;
  std::cout << "sum_weights2  = " << sum_weights2 << std::endl;
  */
  
  // Compute grid with the weighted sum of the neighbors
  double numer = 0.;
  for (int i=0; i<numCells_; i++)
  {
    float grid_val_std = data_array_standardized.at(i);
    float grid_val_std_sum_neighbor = data_array_standardized_smooth.at(i) - central_weight*grid_val_std;
    numer += grid_val_std * grid_val_std_sum_neighbor;
  }

  // Compute Moran's index
  const double denom = sum_weights * (double(numCells_) - 1.0);
  double result = numer / (denom + 1e-6);
  std::cout << "Grid3D: computeMoranIndexGaussianWeights. I = " << result << std::endl;
  
  return result;
}
