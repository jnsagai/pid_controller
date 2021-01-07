#include "PID.h"
#include <vector>
#include <numeric>
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  prev_cte = 0.0;
}

void PID::UpdateError(double cte)
{
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = cte;
  i_error += cte;
  d_error = cte - prev_cte;
  prev_cte = cte;
}

double PID::TotalError()
{
  /**
   * TODO: Calculate and return the total error
   */
  return p_error * Kp + i_error * Ki + d_error * Kd;
}

void PID::SetGains(const double Kp_G, const double Ki_G, const double Kd_G)
{
  Kp = Kp_G;
  Ki = Ki_G;
  Kd = Kd_G;
}

double PID::CalcSteerAngle(double Kp_, double Ki_, double Kd_)
{
  double angle;

  angle = -Kp_ * p_error - Kd_ * d_error - Ki_ * i_error;

  return angle;
}