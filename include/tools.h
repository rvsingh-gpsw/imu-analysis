#pragma once

#include <vector>

namespace imua
{

  /**
   * Compute the norm of the (x,y,z) values.
   * @param x     first array
   * @param y     second array
   * @param z     third array
   * @param n     number of value in x,y, and z arrays
   * @param norm  output array
   */
  void ComputeNorm(const float * x,
                   const float * y,
                   const float * z,
                   const int     n,
                   std::vector<float> & norm);


  /**
   * Smooth an array using 2 pass:
   *   y[i] = weight * x[i] + (1-weight) * y[i-1]
   *   y[i] = weight * y[i] + (1-weight) * y[i+1]
   * @param input   input array
   * @param size    number of value in the input / output array
   * @param output  output array
   * @param weight  weight parameter
   */
  void SmoothArray(const float * input,
                   const int     size,
                   std::vector<float> & output,
                   const float weight = 0.1f);


  /**
   * Smooth an array using 2 pass:
   *   y[i] = weight * x[i] + (1-weight) * y[i-1]
   *   y[i] = weight * y[i] + (1-weight) * y[i+1]
   * @param input   input array
   * @param output  output array
   * @param weight  weight parameter
   */
  void SmoothArray(const std::vector<float> & input,
                   std::vector<float> & output,
                   const float weight = 0.1f);

  /**
   * Inplace smooth an array using 2 pass:
   *   y[i] = weight * x[i] + (1-weight) * y[i-1]
   *   y[i] = weight * y[i] + (1-weight) * y[i+1]
   * @param array   array to smooth
   * @param weight  weight parameter
   */
  void SmoothArray(const std::vector<float> & array,
                   const float weight = 0.1f);



}
