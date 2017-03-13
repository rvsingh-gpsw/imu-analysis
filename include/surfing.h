#pragma once

#include <types.h>
#include <vector>

namespace imua
{
  namespace surfing
  {

    /**
     * This is a dummy function. It uses the one of generic just to try the structure.
     */
    bool detectSurfing(const IMU & imu, std::vector<Detection> & detections, float min_surf_time);


  }
}
