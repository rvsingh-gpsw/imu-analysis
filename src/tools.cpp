#include "tools.h"
#include <cmath>

namespace imua
{

  void ComputeNorm(const float * x,
                   const float * y,
                   const float * z,
                   const int     n,
                   std::vector<float> & norm)
  {

    // Allocate memory
    norm.clear();
    norm.resize(n);

    // Compute the norm
    for (int i=0; i<n; ++i) {
      norm[i] = std::sqrt(x[i]*x[i] + y[i]*y[i] + z[i]*z[i]);
    }
  }




  void SmoothArray(const std::vector<float> & input,
                   std::vector<float> & output,
                   const float weight)
  {

    // Allocate the memory
    output.clear();
    output.resize(input.size());

    // Parameters
    const float alpha = weight;
    const float beta  = 1.f - weight;

    // Forward smoothing
    output[0] = input[0];
    for (int i=1; i<input.size(); ++i) {
        output[i] = alpha * input[i] + beta * output[i-1];
    }

    // Backward smoothing
    for (int i=input.size()-2; i>=0; --i) {
        output[i] = alpha * output[i] + beta * output[i+1];
    }
  }


  void SmoothArray(const float * input,
                   const int     size,
                   std::vector<float> & output,
                   const float weight)
  {

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


  void SmoothArray(std::vector<float> & array,
                   const float weight = 0.1f)
  {
    std::vector<float> tmp;
    SmoothArray(array, tmp, weight);
    std::swap(array, tmp);
  }
}
