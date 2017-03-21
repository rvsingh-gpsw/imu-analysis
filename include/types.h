#pragma once

#include <string>


namespace imua
{


  /**
   *
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
   *
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
   *
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
    float climax;            // climax time
    float value;             // value for the current detection
    std::string description; // description for the current detection

    Detection();

    Detection(const float start,
              const float end,
              const std::string description);

    Detection(const float start,
              const float end,
              const float climax,
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
    //initialize values
    float  roll_init;
    float  pitch_init;
    float  yaw_init;
    //do an initializea
    int do_init;
  };
}
