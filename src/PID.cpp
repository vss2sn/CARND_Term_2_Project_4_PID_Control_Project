#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  cout<<"PID object created"<<endl;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  p_error = 0.0;
  i_err = 0.0;
  d_error = 0.0;

  prev_cte = 0.0;

  count = 0;
  c_err = 0.0;
  // TODO: Set more reasonable limits if required
  min_error = std::numeric_limits<double>::max();
  max_error = std::numeric_limits<double>::min();
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_err += cte;
  #ifdef SMOOTHER
  if(fabs(Ki*i_err)>0.03) i_err=i_err/fabs(i_err)*0.03/Ki;
  #endif
  d_error = cte - prev_cte;
  prev_cte = cte;

  c_err += cte;
  count++;

  if (cte > max_error) max_error = cte;
  if (cte < min_error) min_error = cte;
}

double PID::TotalError() {
  total = ((p_error * Kp) + (i_err * Ki) + (d_error * Kd));
  #ifdef SMOOTHER
  if (fabs(total)>1.0) total = total/fabs(total)*1.0;
  #endif
  return total;
}
