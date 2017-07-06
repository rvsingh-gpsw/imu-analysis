#include "tools.h"
#include <cmath>
#include <iostream>

// Local functions
namespace {
    
    /**
     * Box filter.
     * @param input  input array
     * @param size   number of value in the input / output arrays
     * @param output output array
     * @param sigma  radius of the box filter
     */
    void BoxFilter(const float * input,
                   const int     size,
                   float       * output,
                   const int     radius) {
        
        for (int i=0; i<size; ++i) {
            const int start = std::max(i-radius, 0);
            const int end   = std::min(i+radius, size-1);
            const int nb    = end-start+1;
            float sum = 0.f;
            for (int j=start; j<=end; ++j) {
                sum += input[j];
            }
            output[i] = sum / nb;
        }
    }
}


namespace imua
{


    bool ComputeNorm(const float * x,
                     const float * y,
                     const float * z,
                     const int     size,
                     float       * norm) {

        // Sanity check
        if (size<1)
          return false;

        // Sanity check
        if (x==NULL || y==NULL || z==NULL || norm==NULL)
          return false;

        // Compute Euclidean norm
        for (int i=0; i<size; ++i) {
            norm[i] = std::sqrt(x[i]*x[i] + y[i]*y[i] + z[i]*z[i]);
        }


        return true;
    }


    bool SmoothArrayStream(const float * input,
                           const int     size,
                           float       * output,
                           const float   weight) {

        // Sanity check
        if (size<1)
          return false;

        // Sanity check
        if (input==NULL || output==NULL)
          return false;

        // Sanity check
        if (weight<0.f || weight>1.f)
          return false;
        
        // Parameters
        const float alpha = weight;
        const float beta  = 1.f - weight;
        
        // Forward smoothing
        output[0] = input[0];
        for (int i=1; i<size; ++i) {
            output[i] = alpha * input[i] + beta * output[i-1];
        }
        
        // Backward smoothing
        for (int i=size-2; i>=0; --i) {
            output[i] = alpha * output[i] + beta * output[i+1];
        }

        return true;
    }
    
    
    bool SmoothArrayBoxFilter(const float * input,
                              const int     size,
                              float       * output,
                              const float   sigma) {

        // Sanity check
        if (size<1)
          return false;

        // Sanity check
        if (input==NULL || output==NULL)
          return false;

        // Sanity check
        if (sigma<=0.f)
          return false;
        
        try {

          // Allocate temporary buffer
          std::vector<float> buffer(size);
          
          // Compute kernel radius
          const int k = 3;
          const int r = std::ceil(std::sqrt(sigma*sigma*12/k + 1.f));
          
          // Apply 3 times the box filter
          BoxFilter(input,      size, output,     r);
          BoxFilter(output,     size, &buffer[0], r);
          BoxFilter(&buffer[0], size, output,     r);
          
        }
        catch (...) {
          std::cerr << "Exception caught in SmoothArrayBoxFilter" << std::endl;
          return false;
        }

        return true;
    }


}
