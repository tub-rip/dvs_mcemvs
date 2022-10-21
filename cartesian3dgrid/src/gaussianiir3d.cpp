/**
 * \file gaussianiir3d.c
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

#include <math.h>

#include "cartesian3dgrid/gaussianiir3d.h"

/**
 * \brief Fast 3D Gaussian convolution IIR approximation
 * \param volume the data volume, modified in-place
 * \param width, height, depth the data dimensions
 * \param sigma the standard deviation of the Gaussian in pixels
 * \param numsteps number of timesteps, more steps implies better accuracy
 *
 * Implements the fast Gaussian convolution algorithm of Alvarez and Mazorra,
 * where the Gaussian is approximated by a cascade of first-order infinite 
 * impulsive response (IIR) filters.  Boundaries are handled with half-sample
 * symmetric extension.
 * 
 * Gaussian convolution is approached as approximating the heat equation and 
 * each timestep is performed with an efficient recursive computation.  Using
 * more steps yields a more accurate approximation of the Gaussian.  A 
 * reasonable default value for \c numsteps is 4.
 * 
 * The data is assumed to be ordered such that
 *   volume[x + width*(y + height*z)] = data value at (x,y,z).
 *
 * Reference:
 * Alvarez, Mazorra, "Signal and Image Restoration using Shock Filters and
 * Anisotropic Diffusion," SIAM J. on Numerical Analysis, vol. 31, no. 2, 
 * pp. 590-605, 1994.
 */
void gaussianiir3d(float *volume, long width, long height, long depth, float sigma, int numsteps)
{
    const long plane = width*height;
    const long numel = plane*depth;
    double lambda, dnu;
    float nu, boundaryscale, postscale;
    float *ptr;
    long i, x, y, z;
    int step;
    
    if(sigma <= 0 || numsteps < 0)
        return;
    
    lambda = (sigma*sigma)/(2.0*numsteps);
    dnu = (1.0 + 2.0*lambda - sqrt(1.0 + 4.0*lambda))/(2.0*lambda);
    nu = (float)dnu;
    boundaryscale = (float)(1.0/(1.0 - dnu));
    postscale = (float)(pow(dnu/lambda,3*numsteps));
    
    /* Filter horizontally along each row */
    for(z = 0; z < depth; z++)
    {
        for(y = 0; y < height; y++)
        {
            for(step = 0; step < numsteps; step++)
            {
                ptr = volume + width*(y + height*z);
                ptr[0] *= boundaryscale;
                
                /* Filter rightwards */
                for(x = 1; x < width; x++)
                    ptr[x] += nu*ptr[x - 1];
                
                ptr[x = width - 1] *= boundaryscale;
                
                /* Filter leftwards */
                for(; x > 0; x--)
                    ptr[x - 1] += nu*ptr[x];
            }
        }
    }
    
    /* Filter vertically along each column */
    for(z = 0; z < depth; z++)
    {
        for(x = 0; x < width; x++)
        {
            for(step = 0; step < numsteps; step++)
            {
                ptr = volume + x + plane*z;
                ptr[0] *= boundaryscale;
                
                /* Filter downwards */
                for(i = width; i < plane; i += width)
                    ptr[i] += nu*ptr[i - width];
                
                ptr[i = plane - width] *= boundaryscale;
                
                /* Filter upwards */
                for(; i > 0; i -= width)
                    ptr[i - width] += nu*ptr[i];
            }
        }
    }
    
    /* Filter along z-dimension */
    for(y = 0; y < height; y++)
    {
        for(x = 0; x < width; x++)
        {
            for(step = 0; step < numsteps; step++)
            {
                ptr = volume + x + width*y;
                ptr[0] *= boundaryscale;
                
                for(i = plane; i < numel; i += plane)
                    ptr[i] += nu*ptr[i - plane];
                
                ptr[i = numel - plane] *= boundaryscale;
                
                for(; i > 0; i -= plane)
                    ptr[i - plane] += nu*ptr[i];
            }
        }
    }
    
    for(i = 0; i < numel; i++)
        volume[i] *= postscale;
    
    return;
}


// Allow for a different sigma in each dimension
void gaussianiir3d(float *volume, long width, long height, long depth, float sigma_x, float sigma_y, float sigma_z, int numsteps)
{
    const long plane = width*height;
    const long numel = plane*depth;
    double lambda, dnu;
    float nu, boundaryscale, postscale = 1.f;
    float *ptr;
    long i, x, y, z;
    int step;
    
    if(sigma_x <= 0 || sigma_y <= 0 || sigma_z <= 0 || numsteps < 0)
        return;

    /* Filter horizontally along each row */
    
    lambda = (sigma_x*sigma_x)/(2.0*numsteps);
    dnu = (1.0 + 2.0*lambda - sqrt(1.0 + 4.0*lambda))/(2.0*lambda);
    nu = (float)dnu;
    boundaryscale = (float)(1.0/(1.0 - dnu));
    float postscale_x = (float)(pow(dnu/lambda,numsteps));
    postscale *= postscale_x;

    for(z = 0; z < depth; z++)
    {
        for(y = 0; y < height; y++)
        {
            for(step = 0; step < numsteps; step++)
            {
                ptr = volume + width*(y + height*z);
                ptr[0] *= boundaryscale;
                
                /* Filter rightwards */
                for(x = 1; x < width; x++)
                    ptr[x] += nu*ptr[x - 1];
                
                ptr[x = width - 1] *= boundaryscale;
                
                /* Filter leftwards */
                for(; x > 0; x--)
                    ptr[x - 1] += nu*ptr[x];
            }
        }
    }

    /* Filter vertically along each column */
    
    lambda = (sigma_y*sigma_y)/(2.0*numsteps);
    dnu = (1.0 + 2.0*lambda - sqrt(1.0 + 4.0*lambda))/(2.0*lambda);
    nu = (float)dnu;
    boundaryscale = (float)(1.0/(1.0 - dnu));
    float postscale_y = (float)(pow(dnu/lambda,numsteps));
    postscale *= postscale_y;
    
    for(z = 0; z < depth; z++)
    {
        for(x = 0; x < width; x++)
        {
            for(step = 0; step < numsteps; step++)
            {
                ptr = volume + x + plane*z;
                ptr[0] *= boundaryscale;
                
                /* Filter downwards */
                for(i = width; i < plane; i += width)
                    ptr[i] += nu*ptr[i - width];
                
                ptr[i = plane - width] *= boundaryscale;
                
                /* Filter upwards */
                for(; i > 0; i -= width)
                    ptr[i - width] += nu*ptr[i];
            }
        }
    }
    
    /* Filter along z-dimension */

    lambda = (sigma_z*sigma_z)/(2.0*numsteps);
    dnu = (1.0 + 2.0*lambda - sqrt(1.0 + 4.0*lambda))/(2.0*lambda);
    nu = (float)dnu;
    boundaryscale = (float)(1.0/(1.0 - dnu));
    float postscale_z = (float)(pow(dnu/lambda,numsteps));
    postscale *= postscale_z;
    
    for(y = 0; y < height; y++)
    {
        for(x = 0; x < width; x++)
        {
            for(step = 0; step < numsteps; step++)
            {
                ptr = volume + x + width*y;
                ptr[0] *= boundaryscale;
                
                for(i = plane; i < numel; i += plane)
                    ptr[i] += nu*ptr[i - plane];
                
                ptr[i = numel - plane] *= boundaryscale;
                
                for(; i > 0; i -= plane)
                    ptr[i - plane] += nu*ptr[i];
            }
        }
    }
    
    for(i = 0; i < numel; i++)
        volume[i] *= postscale;
    
    return;
}
