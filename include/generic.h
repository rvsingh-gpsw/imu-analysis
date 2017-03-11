#pragma once

#include <types.h>

#include <vector>

namespace imua
{
  namespace generic
  {

    /**
     *
     */
    bool detectJumps(const float durationMin, std::vector<Detection> & detection);

  }
}
