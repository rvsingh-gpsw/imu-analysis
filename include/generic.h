#pragma once

#include "types.h"
#include <vector>

namespace imua
{
  namespace generic
  {

    /**
     * Detect jumps.
     * @param imu                 IMU structure
     * @param detections          Output array that will contain detections
     * @param thresholdNorm       Acceleration (in m/s^2) value under which a jump is detected
     * @param hangetimeThreshold  Minimum duration of a jump
     * @return true is everything went fine, false otherwise
     */
    bool detectJumps(const IMU              & imu,
                     std::vector<Detection> & detections,
                     const float              thresholdNorm = 4.f,
                     const float              durationMin = 0.25f);

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
                     const int secant_length = 100,
                     const float threshold_spin_degrees = 90.f,
                     const int threshold_samples = 10);

    /**
     * Default settings for mountain bike
     */
    void detectCorners(const IMU & imu,
                       const Euler & euler,
                       std::vector<Detection> & detections,
                       const int secant_length=400,
                       const float threshold_spin_degrees=20.f,
                       const int threshold_samples = 300);

  }
}
