#ifndef PID_H
#define PID_H

#include <iosfwd>

class PID {
 public:
  PID();

  void Init(double Kp, double Ki, double Kd);

  void UpdateError(double cte, double dt);

  double GetControl();

  void ResetState();

  friend std::ostream& operator<<(std::ostream& os, const PID& pid);

 private:
  double p_error_;
  double i_error_;
  double d_error_;

  double kp_;
  double ki_;
  double kd_;
};

#endif /* PID_H */
