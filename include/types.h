#pragma once

#include <string>

namespace imua
{


  /**
   * Gyroscope structure
   */
  struct Gyroscope
  {
    float   samplingRate;
    int     size;
    float * t;
    float * x;
    float * y;
    float * z;

    Gyroscope();
  };

  /**
   * Accelerometer structure
   */
  struct Accelerometer
  {
    float   samplingRate;
    int     size;
    float * t;
    float * x;
    float * y;
    float * z;

    Accelerometer();
  };

  /**
   * IMU structure
   */
  struct IMU
  {
    Gyroscope     gyro;
    Accelerometer accl;
  };


  /**
   * Detection
   */
  struct Detection
  {

    float start;             // starting time
    float end;               // ending time
    float value;             // value, especially useful for multi-level detections
    std::string description; // description, mainly for debugging

    /**
     * Constructor
     */
    Detection();

    /**
     * Constructor
     * @param start       starting time
     * @param end         ending time
     * @param description description
     */
    Detection(const float start,
              const float end,
              const std::string description);

    /**
     * Constructor
     * @param start       starting time
     * @param end         ending time
     * @param value       value
     * @param description description
     */
    Detection(const float start,
              const float end,
              const float value,
              const std::string description);
  };


  /**
   *
   */
  struct Euler
  {
    float sampling_rate;
    int    num_samples;
    float * t;
    float * roll;
    float * pitch;
    float * yaw;

    //mean position values
    float  roll_mean;
    float  pitch_mean;
    float  yaw_mean;
    //do an initializea
    int do_init;
  };
}
