#include "generic.h"

#include <iostream>

namespace imua
{
  namespace generic
  {

    bool detectJumps(std::vector<Detection> & detections,
                     const float durationMin)
    {
      //std::cout << "Generic jump with duration of " << durationMin << " minimum" << std::endl;
      detections.push_back(Detection(2., 5.5, "jump"));
      detections.push_back(Detection(10., 20., "jump"));
      detections.push_back(Detection(25., 25.5, "jump"));
      return true;
    }

  }
}
