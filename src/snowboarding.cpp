#include "snowboarding.h"
#include "generic.h"

#include <iostream>

namespace imua
{
  namespace snowboarding
  {

    bool detectJumps(const IMU & imu, std::vector<Detection> & detections)
    {
      return generic::detectJumps(imu, detections, 4.f, 0.2f);
    }
    
  }
}
