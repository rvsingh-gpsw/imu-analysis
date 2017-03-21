#pragma once

#include "types.h"

namespace imua
{

  int  getEulerAngles(const IMU & imu, Euler & euler, int do_euler_init = 0);

}
