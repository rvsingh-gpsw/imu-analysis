#pragma once

#include "types.h"
#include <vector>

namespace imua
{
  namespace skateboarding
  {

    /**
     * Detect jumps.
     * @param imu        IMU structure
     * @param detections output array that will contain detections
     * @return true is everything went fine, false otherwise
     */
    bool detectJumps(const IMU & imu, std::vector<Detection> & detections);

  }
}
