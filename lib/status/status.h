#ifndef __status_h
#define __status_h

#include "mobility_skid.h"

static const uint8_t LED_ROS2_WAITING = 5;
static const uint8_t LED_ROS2_CONNECT = 18;

class Status
{
public:
  Status();
  static bool setup();

  static MobilitySkid *mobility;
};

#endif // __status_h