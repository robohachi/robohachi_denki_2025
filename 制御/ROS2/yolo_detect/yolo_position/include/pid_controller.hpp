#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>

class PIDController
{
public:
  PIDController(double kp = 0.0, double ki = 0.0, double kd = 0.0,
                double min_output = -1.0, double max_output = 1.0)
    : kp_(kp), ki_(ki), kd_(kd),
      min_output_(min_output), max_output_(max_output),
      integral_(0.0), prev_error_(0.0), first_run_(true) {}

  double calculate(double error, double dt)
  {
    if (dt <= 0.0) return 0.0;

    double p_term = kp_ * error;

    integral_ += error * dt;
    double i_term = ki_ * integral_;

    double d_term = 0.0;
    if (!first_run_) {
      d_term = kd_ * (error - prev_error_) / dt;
    }
    first_run_ = false;
    prev_error_ = error;

    double output = p_term + i_term + d_term;
    return std::clamp(output, min_output_, max_output_);
  }

  void reset()
  {
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_run_ = true;
  }

  void setGains(double kp, double ki, double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void setOutputLimits(double min_output, double max_output)
  {
    min_output_ = min_output;
    max_output_ = max_output;
  }

private:
  double kp_, ki_, kd_;
  double min_output_, max_output_;
  double integral_;
  double prev_error_;
  bool first_run_;
};

#endif
