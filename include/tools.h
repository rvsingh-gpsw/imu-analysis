#pragma once

#include <vector>

namespace imua
{

    /**
     * Compute the norm of the (x,y,z) values.
     * @param x     first array
     * @param y     second array
     * @param z     third array
     * @param size  number of value in the input / output arrays
     * @param norm  output array
     * @return true if everything is fine, false otherwise
     */
    bool ComputeNorm(const float * x,
                     const float * y,
                     const float * z,
                     const int     size,
                     float       * norm);

    /**
     * Smooth an array using 2 passes (foreward and backward):
     *   y[i] = weight * x[i] + (1-weight) * y[i-1]
     *   y[i] = weight * y[i] + (1-weight) * y[i+1]
     * @param input   input array
     * @param size    number of value in the input / output arrays
     * @param output  output array
     * @param weight  weight parameter
     * @return true if everything is fine, false otherwise
     */
    bool SmoothArrayStream(const float * input,
                           const int     size,
                           float       * output,
                           const float   weight = 0.1f);

    /**
     * Smooth an array using 3 pass box filter.
     * The distance between 2 consecutive values in the input array is assumed to be 1.
     * The standard deviation is to be adapted based on this assumption.
     * @param input  input array
     * @param size   number of value in the input / output arrays
     * @param output output array
     * @param sigma  standard deviation
     * @return true if everything is fine, false otherwise
     */
    bool SmoothArrayBoxFilter(const float * input,
                              const int     size,
                              float       * output,
                              const float   sigma = 1.f);

}
