#pragma once

#include "types.h"
#include <vector>

namespace imua
{
  namespace surfing
  {

    /**
     * 
     */
    bool detectSurfing(const IMU & imu, std::vector<Detection> & detections, float min_surf_time);


  }
}
