#include "tools.h"
#include <cmath>

namespace imua
{

  void ComputeNorm(const float * x,
                   const float * y,
                   const float * z,
                   const int     n,
                   std::vector<float> & norm) {

    // Allocate memory
    norm.clear();
    norm.resize(n);

    // Compute the norm
    for (int i=0; i<n; ++i) {
      norm[i] = std::sqrt(x[i]*x[i] + y[i]*y[i] + z[i]*z[i]);
    }
  }


  void SmoothArray(const float        * input,
                   const int            size,
                   std::vector<float> & output,
                   const float          weight) {

    // Allocate the memory
    output.clear();
    output.resize(size);

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
  }


  void SmoothArray(const std::vector<float> & input,
                   std::vector<float>       & output,
                   const float                weight) {
    SmoothArray(&input[0], input.size(), output, weight);
  }


  void SmoothArray(std::vector<float> & array,
                   const float          weight) {
    std::vector<float> tmp;
    SmoothArray(array, tmp, weight);
    std::swap(array, tmp);
  }


  void BoxFilter(const float * input,
                 float       * output,
                 const int     size,
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


  void SmoothArrayBox(const float * input,
                      float       * output,
                      const int     size,
                      const float   sigma)
  {

    // Allocate temporary buffer
    std::vector<float> buffer(size);

    // Compute kernel radius
    const int k = 3;
    const int r = std::ceil(std::sqrt(sigma*sigma*12/k + 1.f));

    // Apply 3 times the box filter
    BoxFilter(input,  output,     size, r);
    BoxFilter(output, &buffer[0], size, r);
    BoxFilter(&buffer[0], output, size, r);
  }


  void SmoothArrayBox(const std::vector<float> & input,
                      std::vector<float>       & output,
                      const float                sigma) {

    // Allocate memory
    const int n = input.size();
    output.resize(n);

    // Allocate temporary buffer
    std::vector<float> buffer(n);

    // Compute kernel radius
    const int k = 3;
    const int r = std::ceil(std::sqrt(sigma*sigma*12/k + 1.f));

 
    // Apply 3 times the box filter
    BoxFilter(&input[0],  &output[0], n, r);
    BoxFilter(&output[0], &buffer[0], n, r);
    BoxFilter(&buffer[0], &output[0], n, r);
    
  }


}
