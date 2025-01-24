#include "micro_rosso.h"

#include "status.h"

MobilitySkid *Status::mobility;

Status::Status() {
};

static void ros_state_cb(ros_states state)
{
  switch (state)
  {
  case WAITING_AGENT:
    digitalWrite(LED_ROS2_WAITING, 1);
    digitalWrite(LED_ROS2_CONNECT, 0);
    break;
  case AGENT_AVAILABLE:
    digitalWrite(LED_ROS2_WAITING, 1);
    digitalWrite(LED_ROS2_CONNECT, 1);
    break;
  case AGENT_CONNECTED:
    digitalWrite(LED_ROS2_WAITING, 0);
    digitalWrite(LED_ROS2_CONNECT, 1);
    break;
  case AGENT_DISCONNECTED:
    digitalWrite(LED_ROS2_WAITING, 0);
    digitalWrite(LED_ROS2_CONNECT, 0);
    if (Status::mobility)
    {
      Status::mobility->stop();
      D_println("Status: emergency stop on ROS disconnect.");
    }
    break;
  default:
    break;
  }
}

bool Status::setup()
{
  D_print("setup: status... ");

  pinMode(LED_ROS2_WAITING, OUTPUT);
  pinMode(LED_ROS2_CONNECT, OUTPUT);
  digitalWrite(LED_ROS2_WAITING, 0);
  digitalWrite(LED_ROS2_CONNECT, 0);

  micro_rosso::ros_state_listeners.push_back(ros_state_cb);

  D_println("done.");
  return true;
}
