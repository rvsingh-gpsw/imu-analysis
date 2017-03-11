#include "generic.h"

#include <iostream>

namespace imua
{
  namespace generic
  {

    bool detectJumps(const float durationMin, std::vector<Detection> & detection)
    {
      std::cout << "Generic jump with duration of " << durationMin << " minimum" << std::endl;
      return true;
    }

  }
}
