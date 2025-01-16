#include "micro_rosso.h"

#include "tf_publisher.h"
#include <geometry_msgs/msg/transform_stamped.h>

timer_descriptor *timer_announce = new timer_descriptor;

static publisher_descriptor pdescriptor_tf_static;

rmw_qos_profile_t qos_profile_static = rmw_qos_profile_default;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      return false;              \
    }                            \
  }
#define RCNOCHECK(fn)       \
  {                         \
    rcl_ret_t temp_rc = fn; \
    (void)temp_rc;          \
  }

TF_publisher::TF_publisher()
{
  qos_profile_static.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos_profile_static.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  qos_profile_static.depth = 1;
  qos_profile_static.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

static void timer_handler_announce(rcl_timer_t *timer, int64_t last_call_time)
{

  //TODO send from here like this:
  //RCNOCHECK(rcl_publish(&pdescriptor_tf_static.publisher, &msg_announce, NULL));

  //one off, cancel timer
  RCNOCHECK(rcl_timer_cancel(timer));
  delete timer_announce;

  return;
}

bool TF_publisher::setup(const char *topic_name)
{
  D_println("setup: tf_publisher");

  pdescriptor_tf_static.qos = QOS_CUSTOM;
  pdescriptor_tf_static.qos_profile = &qos_profile_static;
  pdescriptor_tf_static.type_support =
      (rosidl_message_type_support_t *)ROSIDL_GET_MSG_TYPE_SUPPORT(
          geometry_msgs, msg, TransformStamped);
  pdescriptor_tf_static.topic_name = topic_name;
  micro_rosso::publishers.push_back(&pdescriptor_tf_static);

  // topic to be sent after initialization completed
  timer_announce->timeout_ns = RCL_MS_TO_NS(1000);
  timer_announce->timer_handler = timer_handler_announce;
  micro_rosso::timers.push_back(timer_announce);

  return true;
}
