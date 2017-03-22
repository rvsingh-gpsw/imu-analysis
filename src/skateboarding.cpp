#include "skateboarding.h"
#include "generic.h"

#include <iostream>

namespace imua
{
  namespace skateboarding
  {

    bool detectJumps(const IMU & imu, std::vector<Detection> & detections)
    {
      return generic::detectJumps(imu, detections, 4.f, 0.2f);
    }
    
  }
}
