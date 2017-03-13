#include "snowboard.h"
#include "generic.h"

#include <iostream>

namespace imua
{
  namespace snowboard
  {

    bool detectJumps(const IMU & imu, std::vector<Detection> & detections)
    {
      return generic::detectJumps(imu, detections, 4.f, 0.2f);
    }

    bool detect360(const IMU & imu, std::vector<Detection> & detections)
    {
      detections.push_back(Detection(12., 15., "360"));
      detections.push_back(Detection(16., 23.33, "360"));
      detections.push_back(Detection(28.45, 32.16, "360"));
      return true;
    }

  }
}
