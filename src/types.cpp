#include "types.h"

namespace imua
{

  Detection::Detection():
  start(0.f),
  end(0.f),
  climax(0.f),
  value(0.f),
  description("Default"){}


  Detection::Detection(const float startTime,
                       const float endTime,
                       const std::string description):
  start(startTime),
  end(endTime),
  climax(startTime),
  value(0.f),
  description(description){}


}
