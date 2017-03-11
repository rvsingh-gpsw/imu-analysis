#include "snowboard.h"
#include "generic.h"

#include <iostream>

namespace imua
{
  namespace snowboard
  {

    bool detectJumps(std::vector<Detection> & detection)
    {
      return generic::detectJumps(0.66f, detection);
    }

  }
}
