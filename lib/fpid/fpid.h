#ifndef __fpid_h
#define __fpid_h

class Fpid
{

public:
  Fpid();
  Fpid(float min_out, float max_out, float kf, float kp, float ki, float kd);

  float compute(float setpoint, float measure, float time_step);

  void update_k_independent(float kf, float kp, float ki, float kd);
  void update_k_dependent_ideal(float kf, float kc, float Ti, float Td);

  void reset_errors(float measure = 0.0);

  float min_out;
  float max_out;

  float kf; // forward controller
  float kp;
  float ki;
  float kd;

private:
  float ki_term;
  float prev_measure;
};

#endif // __fpid_h
