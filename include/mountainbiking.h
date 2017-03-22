#pragma once

#include "types.h"
#include <vector>

namespace imua
{
  namespace mountainbiking
  {

    /**
     * Detect jumps.
     * @param imu        IMU structure
     * @param detections output array that will contain detections
     * @return true is everything went fine, false otherwise
     */
    bool detectJumps(const IMU & imu, std::vector<Detection> & detections);

    /**
     * Detect flips.
     * @param imu        IMU structure
     * @param detections output array that will contain detections
     * @return true is everything went fine, false otherwise
     */
    void detectFlips(const IMU & imu,
                     const Euler & euler,
                     std::vector<Detection> & detections);

    /**
     * Detect corners.
     * @param imu        IMU structure
     * @param detections output array that will contain detections
     * @return true is everything went fine, false otherwise
     */
    void detectCorners(const IMU & imu,
                       const Euler & euler,
                       std::vector<Detection> & detections);

  }
}
