#ifndef __mobility_skid_h
#define __mobility_skid_h

#define SKID_TOPIC_CMD_VEL "/cmd_vel"

class MobilitySkid
{
public:
  MobilitySkid();

  static bool setup();

  static void set_motor_enable(bool enable); // TODO
  static void stop();
};

#endif // __mobility_skid_h
