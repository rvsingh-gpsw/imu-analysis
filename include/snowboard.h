#pragma once

#include <types.h>

#include <vector>

namespace imua
{
  namespace snowboard
  {

    /**
     *
     */
    bool detectJumps(std::vector<Detection> & detections);

    /**
     *
     */
    bool detect360(std::vector<Detection> & detections);

  }
}
