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
  climax(0.f),
  value(0.f),
  description("Default"){}


  Detection::Detection(const float start,
                       const float end,
                       const std::string description):
  start(start),
  end(end),
  climax(start),
  value(0.f),
  description(description){}


}
