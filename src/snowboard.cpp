#include "snowboard.h"
#include "generic.h"

#include <iostream>

namespace imua
{
  namespace snowboard
  {

    bool detectJumps(std::vector<Detection> & detections)
    {
      return generic::detectJumps(detections, 0.66f);
    }

    bool detect360(std::vector<Detection> & detections)
    {
      detections.push_back(Detection(12., 15., "360"));
      detections.push_back(Detection(16., 23.33, "360"));
      detections.push_back(Detection(28.45, 32.16, "360"));
      return true;
    }

  }
}
