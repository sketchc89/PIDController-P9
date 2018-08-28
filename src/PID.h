#ifndef PID_H
#define PID_H

class PID {
public:
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

  void TunePID();
  void EnableTuning();
  void DisableTuning();
  bool IsTuningOn();
private:
  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;
  
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Tuning parameters
  */
  bool tuning_;
  double p_tuning_;
  double i_tuning_;
  double d_tuning_;
};

#endif /* PID_H */
