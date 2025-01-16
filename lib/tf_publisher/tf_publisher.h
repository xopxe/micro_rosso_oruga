#ifndef __tf_publisher_h
#define __tf_publisher_h

class TF_publisher
{
public:
  TF_publisher();
  static bool setup(const char *topic_name = "/tf_static");
};

#endif // __tf_publisher_h
