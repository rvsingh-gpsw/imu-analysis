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
    bool detectJumps(std::vector<Detection> & detections,
                     const float durationMin=0.5f);

  }
}
