#include "surfing.h"
#include "generic.h"

#include <iostream>

namespace imua
{
  namespace surfing
  {

    bool detectSurfing(const IMU & imu, std::vector<Detection> & detections)
    {
      return 0; //generic::detectJumps(imu, detections, 4.f, 0.2f);
    }

  }
}
