#pragma once
#include "cartesian3dgrid.h"

/**
 * \file gaussianiir3d.h
 * \brief Fast 3D Gaussian convolution IIR approximation
 * \author Pascal Getreuer <getreuer@gmail.com>
 * 
 * Copyright (c) 2011, Pascal Getreuer
 * All rights reserved.
 * 
 * This program is free software: you can redistribute it and/or modify it
 * under, at your option, the terms of the GNU General Public License as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version, or the terms of the 
 * simplified BSD license.
 *
 * You should have received a copy of these licenses along with this program.
 * If not, see <http://www.gnu.org/licenses/> and
 * <http://www.opensource.org/licenses/bsd-license.html>.
 */

void gaussianiir3d(float *volume, long width, long height, long depth, float sigma, int numsteps);

void gaussianiir3d(float *volume, long width, long height, long depth, float sigma_x, float sigma_y, float sigma_z, int numsteps);

void smoothInPlace(const float sigma=1.f);
void smoothInPlaceIIR3D(const float sigma_x=1.f, const float sigma_y=1.f, const float sigma_z=1.f);
void laplacianInPlace();
