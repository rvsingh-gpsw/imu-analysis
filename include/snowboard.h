#pragma once

#include "types.h"
#include <vector>

namespace imua
{
  namespace snowboard
  {

    /**
     * This is a dummy function. It uses the one of generic just to try the structure.
     */
    bool detectJumps(const IMU & imu, std::vector<Detection> & detections);

    /**
     * This is a dummy function.
     */
    bool detect360(const IMU & imu, std::vector<Detection> & detections);

  }
}
