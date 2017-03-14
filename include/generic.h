#pragma once

#include <types.h>

#include <vector>

namespace imua
{
  namespace generic
  {

    bool test();

    /**
     *
     */
    bool detectJumps(const IMU & imu,
                     std::vector<Detection> & detections,
                     const float gforceThreshold = 6.f,
                     const float hangetimeThreshold = 0.25f);

  }
}
