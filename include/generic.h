#pragma once

#include <types.h>

#include <vector>

namespace imua
{
  namespace generic
  {

    /**
     *
     */
    bool detectJumps(std::vector<Detection> & detections,
                     const float durationMin=0.5f);

    bool detectJumps(const IMU & imu,
                     std::vector<Detection> & detections,
                     const float gforceThreshold = 6.f,
                     const float hangetimeThreshold = 0.25f);

  }
}
