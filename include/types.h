#pragma once

#include <string>

namespace imua
{

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

  };
}
