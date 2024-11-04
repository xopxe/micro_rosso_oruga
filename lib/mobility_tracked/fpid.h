#ifndef __fpid_h
#define __fpid_h

class Fpid
{

public:
  Fpid();
  Fpid(float min_out, float max_out, float kf, float kp, float ki, float kd);

  void update_range(float min_out, float max_out);
  void update_k_independent(float kf, float kp, float ki, float kd);
  void update_k_dependent_ideal(float kf, float kc, float Ti, float Td);

  float compute(float setpoint, float measure, float time_step);

  void reset_errors(float measure = 0.0);

private:
  float min_out;
  float max_out;
  float kf; // forward controller
  float kp;
  float ki;
  float kd;

  float ki_term;
  float prev_measure;
};

#endif // __fpid_h
