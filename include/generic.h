#pragma once

#include "types.h"
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

    /**
     *
     */
    void detectFlips(const IMU & imu,
                     const Euler & euler,
                     std::vector<Detection> & detections);

    /**
     *
     */
    void detectSpins(const IMU & imu,
                     const Euler & euler,
                     std::vector<Detection> & detections,
                     const int secant_length=100,
                     const  float threshold_spin_degrees=90.f, int threshold_samples = 10);

    //default settings for mountain bike
    void detectCorners(const IMU & imu,
                     const Euler & euler,
                     std::vector<Detection> & detections,
                     const int secant_length=400,
                     const  float threshold_spin_degrees=20.f, int threshold_samples = 300);


    /**
     *
     */
    void detectShakyParts(const IMU & imu,
                          std::vector<Detection> & detections);

  }
}
