#include "mountainbiking.h"
#include "generic.h"

#include <iostream>

namespace imua
{
  namespace mountainbiking
  {

    bool detectJumps(const IMU & imu, std::vector<Detection> & detections)
    {
      return generic::detectJumps(imu, detections, 9.81f, 0.25f);
    }


    void detectFlips(const IMU & imu,
                     const Euler & euler,
                     std::vector<Detection> & detections)
    {
      generic::detectFlips(imu, euler, detections);
    }


    void detectCorners(const IMU & imu,
                       const Euler & euler,
                       std::vector<Detection> & detections)
    {
      generic::detectCorners(imu, euler, detections, 400, 20.f, 300);
    }
    
  }
}
