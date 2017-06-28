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
     * @param detections          output array that will contain detections
     * @param gforceThreshold     g-force must be under a threshold
     * @param hangetimeThreshold  a jump must be long enough in terms of time
     * @return true is everything went fine, false otherwise
     */
    bool detectJumps(const IMU & imu,
                     std::vector<Detection> & detections,
                     const float gforceThreshold = 6.f,
                     const float hangetimeThreshold = 0.25f);

    bool detectJumps2(const IMU & imu,
                      std::vector<Detection> & detections);

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

    /**
     * Detect camera pans. We only detect left and right pan.
     * The type of the pan  is stored into the "value" field of the detection:
     *   value = 1 -> left pan
     *   value = 2 -> right pan
     * @param imu   IMU structure
     * @param pans  output array that will contain detections
     */
    void detectPans(const IMU & imu,
                    std::vector<Detection> & pans);

  }
}
