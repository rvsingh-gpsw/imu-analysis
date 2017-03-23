#pragma once

#include <vector>

namespace imua
{

  /**
   * Compute the norm of the (x,y,z) values
   */
  void ComputeNorm(const float * x,
                   const float * y,
                   const float * z,
                   const int     n,
                   std::vector<float> & norm);

  /**
   *
   */
  void SmoothArray(const std::vector<float> & input,
                   std::vector<float> & output,
                   const float weight = 0.1f);

}
