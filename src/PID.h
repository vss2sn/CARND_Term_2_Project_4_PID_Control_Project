#ifndef PID_H
#define PID_H

#include <iostream>
#include <limits>
#include <math.h>

//#define SMOOTHER
/*
NOTE:
if "SMOOTHER" defined then larger osscilations, but smoother drive
if not defined then jittery
End NOTE
*/

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_err;
  double d_error;

  /*
  * User defined vars
  */
  long count;     //count
  double prev_cte;   //cumulative error
  double c_err;   //cumulative error
  double min_error;//min value of error
  double max_error;//max value of error
  double total;
  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
