/*
* \file cartesian3dgrid.h
* \brief header file for handling 3d volume
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

#include <vector>
#include <opencv2/core/core.hpp>
#include <iostream>

class Grid3D
{
public:
    Grid3D();
    Grid3D(const unsigned int dimX, const unsigned int dimY, const unsigned int dimZ);
    ~Grid3D();

    void allocate(const unsigned int dimX, const unsigned int dimY, const unsigned int dimZ);
    void deallocate();

    void printInfo() const;

    // The 3D data is stored in a 1D array that is ordered such that
    // volume[x + dimX*(y + dimY*z)] = data value at (x,y,z).

    // Accessing elements of the grid:

    // At integer location
    inline float getGridValueAt(const unsigned int ix, const unsigned int iy, const unsigned int iz) const
    {
        return data_array_.at(ix + size_[0]*(iy + size_[1]*iz));
    }

    inline float getGridValueAt(const unsigned int p) const
    {
        return data_array_.at(p);
    }

    inline void accumulateGridValueAt(const unsigned int p, const float fval)
    {
        data_array_.at(p) += fval;
    }

    inline void setGridValueAt(const unsigned int p, const float fval)
    {
        data_array_.at(p) = fval;
    }

    // At floating point location within a Z slice. Bilinear interpolation or voting.
    inline void accumulateGridValueAt(const float x_f, const float y_f, float* grid);

    // FIXME: The following voxel-wise operations do not use parallelization yet
    inline void addTwoGrids(const Grid3D grid2)
    {
        for (int p = 0; p < numCells_; p++)
            {
                data_array_.at(p) += grid2.data_array_.at(p);
            }
    }

    inline void addInverseOfTwoGrids(const Grid3D grid2, const float eps=1e-2)
    {
        for (int p = 0; p < numCells_; p++)
            {
                data_array_.at(p) = data_array_.at(p) + 1.0f/(eps + grid2.data_array_.at(p));
            }
    }

    inline void computeHMfromSumOfInv(int n)
    {
        for (int p = 0; p < numCells_; p++)
            {
                data_array_.at(p) = (float)n /(data_array_.at(p));
            }
    }
    inline void computeAMfromSum(int n)
    {
        for (int p = 0; p < numCells_; p++)
            {
                data_array_.at(p) = data_array_.at(p)/(float)n;
            }
    }

    inline void subtractTwoGrids(const Grid3D grid2)
    {
        for (int p = 0; p < numCells_; p++)
            {
                data_array_.at(p) -= grid2.data_array_.at(p);
            }
    }

    inline void ratioTwoGrids(const Grid3D grid2, const float eps=1e-1)
    {
        for (int p = 0; p < numCells_; p++)
            {
                data_array_.at(p) /= fabs(grid2.data_array_.at(p)) + eps;
            }
    }

    inline void minTwoGrids(const Grid3D grid2)
    {
        for (int p = 0; p < numCells_; p++)
            {
                data_array_.at(p) = std::min(data_array_.at(p), grid2.data_array_.at(p));
            }
    }

    inline void harmonicMeanTwoGrids(const Grid3D grid2, const float eps=1e-1)
    {
        for (int p = 0; p < numCells_; p++)
            {
                float prod = data_array_.at(p) * grid2.data_array_.at(p);
                float sum  = data_array_.at(p) + grid2.data_array_.at(p);
                data_array_.at(p) = 2*prod / (sum + eps);
            }
    }

    // allows for recursive application by stating number of maps to fuse
    inline void harmonicMeanTwoGrids(const Grid3D grid2, int n, const float eps=1e-1)
    {
        for (int p = 0; p < numCells_; p++)
            {
                float a = data_array_.at(p)/(float)(n-1);
                float prod = a * grid2.data_array_.at(p);
                float sum  = a + grid2.data_array_.at(p);
                data_array_.at(p) = n*prod / (sum + eps);
            }
    }

    inline void rmsTwoGrids(const Grid3D grid2)
    {
        for (int p = 0; p < numCells_; p++)
            {
                float ms = 0.5*(pow(data_array_.at(p), 2) + pow(grid2.data_array_.at(p), 2));
                data_array_.at(p) = sqrt(ms);
            }
    }

    inline void geometricMeanTwoGrids(const Grid3D grid2)
    {
        for (int p = 0; p < numCells_; p++)
            {
                data_array_.at(p) = std::sqrt(data_array_.at(p) * grid2.data_array_.at(p));
            }
    }

    inline void arithmeticMeanTwoGrids(const Grid3D grid2)
    {
        for (int p = 0; p < numCells_; p++)
            {
                data_array_.at(p) = 0.5 * (data_array_.at(p) + grid2.data_array_.at(p));
            }
    }

    inline void quadraticMeanTwoGrids(const Grid3D grid2)
    {
        for (int p = 0; p < numCells_; p++)
            {
                float u = data_array_.at(p);
                float v = grid2.data_array_.at(p);
                data_array_.at(p) = std::sqrt( 0.5 * (u*u + v*v) );
            }
    }

    inline void cubicMeanTwoGrids(const Grid3D grid2)
    {
        for (int p = 0; p < numCells_; p++)
            {
                float u = data_array_.at(p);
                float v = grid2.data_array_.at(p);
                data_array_.at(p) = std::cbrt( 0.5 * (u*u*u + v*v*v) );
            }
    }

    inline void maxTwoGrids(const Grid3D grid2)
    {
        for (int p = 0; p < numCells_; p++)
            {
                data_array_.at(p) = std::max(data_array_.at(p), grid2.data_array_.at(p));
            }
    }


    inline void accumulateZSliceAt(const unsigned int iz, const cv::Mat& img)
    {
        for(unsigned int iy=0; iy < img.rows; iy++)
            {
                for(unsigned int ix=0; ix < img.cols; ix++)
                    {
                        data_array_.at(ix + size_[0]*(iy + size_[1]*iz)) += img.at<float>(iy,ix);
                    }
            }
    }

    cv::Mat getSlice(const unsigned int sliceIdx, const unsigned int dimIdx) const;
    void collapseMaxZSlice(cv::Mat* max_val, cv::Mat* max_pos) const;
    void collapseMinZSlice(cv::Mat* min_val, cv::Mat* min_pos) const;

    void collapseZSliceByGradMag(cv::Mat* confidence, cv::Mat* depth_cell_indices, int half_patchsize=1);
    void collapseZSliceByLaplacianMag(cv::Mat* confidence, cv::Mat* depth_cell_indices);
    void collapseZSliceByDoG(cv::Mat* confidence, cv::Mat* depth_cell_indices);
    void collapseZSliceByLocalVar(cv::Mat* confidence, cv::Mat* depth_cell_indices);
    void collapseZSliceByLocalMeanSquare(cv::Mat* confidence, cv::Mat* depth_cell_indices);

    void computeLocalFocusInPlace(int focus_method);
    void computeLocalMeanSquareInPlace();
    void computeLocalVarInPlace();

    // Statistics
    double computeMeanSquare() const;
    void getMinMax(float* min_val, float* max_val, unsigned long* min_pos=NULL, unsigned long* max_pos=NULL) const;

    void resetGrid();

    // Output
    int writeGridNpy(const char szFilename[]) const;
    void imwriteSlices(const char prefix[], const unsigned int dimIdx, bool normalize_by_minmax=true) const;

    void getDimensions(int* dimX, int* dimY, int* dimZ) const
    {
        *dimX = size_[0];
        *dimY = size_[1];
        *dimZ = size_[2];
    }

    float* getPointerToSlice(int layer)
    {
        return &data_array_.data()[layer * size_[0] * size_[1]];
    }

private:
    std::vector<float> data_array_;
    unsigned int numCells_;
    unsigned int size_[3];

};


// Function implementation

// Bilinear voting within a Z-slice, if point (x,y) is given as float
inline void Grid3D::accumulateGridValueAt(const float x_f, const float y_f, float* grid)
{
    if (x_f >= 0.f && y_f >= 0.f)
        {
            const int x = x_f, y = y_f;
            if (x+1 < size_[0] &&
                    y+1 < size_[1])
                {
                    float* g = grid + x + y * size_[0];
                    const float fx = x_f - x,
                            fy = y_f - y,
                            fx1 = 1.f - fx,
                            fy1 = 1.f - fy;

                    g[0] += fx1*fy1;
                    g[1] += fx*fy1;
                    g[size_[0]]   += fx1*fy;
                    g[size_[0]+1] += fx*fy;
                }
        }
}
