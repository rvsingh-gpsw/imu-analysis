#pragma once

#include <string>

namespace imua
{

  /**
   * Detection
   */
  struct Detection
  {

    float startTime;
    float endTime;
    float climaxTime;
    float climaxValue;
    std::string description;

    Detection();

  };
}
