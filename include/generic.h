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
     * Detect shaky parts in videos. Two levels of shakiness are computed: medium
     * and strong. The level is indicated in the description field of the detection
     * structure in order to keep a simple API (but it might change). Under the low
     * threshold, the camera is not shaky. Above the high threshold, the camera
     * shakiness is strong. Between the two thresholds, the shakiness is medium.
     * The shakiness value is stored into the "value" field of the detection:
     *   value = 1 -> medium
     *   value = 2 -> strong
     * @param imu            IMU structure
     * @param detections     output array that will contain detections
     * @param thresholdLow   low threshold
     * @param thresholdHigh  high threshold
     */
    void detectShakiness(const IMU & imu,
                         std::vector<Detection> & detections,
                         const float thresholdLow=0.5f,
                         const float thresholdHigh=1.f);

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
