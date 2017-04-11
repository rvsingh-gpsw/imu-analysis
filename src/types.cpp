#include "types.h"

namespace imua
{

  Gyroscope::Gyroscope():
  samplingRate(0.f),
  size(0),
  t(NULL),
  x(NULL),
  y(NULL),
  z(NULL){}


  Accelerometer::Accelerometer():
  samplingRate(0.f),
  size(0),
  t(NULL),
  x(NULL),
  y(NULL),
  z(NULL){}


  Detection::Detection():
  start(0.f),
  end(0.f),
  value(1.f),
  description("Default"){}


  Detection::Detection(const float start,
                       const float end,
                       const std::string description):
  start(start),
  end(end),
  value(1.f),
  description(description){}


  Detection::Detection(const float start,
                       const float end,
                       const float value,
                       const std::string description):
  start(start),
  end(end),
  value(value),
  description(description){}


}
