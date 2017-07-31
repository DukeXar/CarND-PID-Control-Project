#include "PID.h"
#include <iostream>

PID::PID() : p_error_(0), i_error_(0), d_error_(0) {}

void PID::Init(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  ResetState();
}

void PID::UpdateError(double cte, double dt) {
  // std::cout << "dt=" << dt << std::endl;
  i_error_ += cte;
  d_error_ = (cte - p_error_) / dt;
  p_error_ = cte;
}

double PID::GetControl() {
  return -kp_ * p_error_ - kd_ * d_error_ - ki_ * i_error_;
}

void PID::ResetState() {
  i_error_ = 0;
  p_error_ = 0;
  d_error_ = 0;
}

std::ostream& operator<<(std::ostream& os, const PID& pid) {
  os << "Kp=" << pid.kp_ << ", Ki=" << pid.ki_ << ", Kd=" << pid.kd_;
  return os;
}