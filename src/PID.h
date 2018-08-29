#ifndef PID_H
#define PID_H
#include <vector>

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
  void DisplayPID();
  void UpdatePID(double Kp, double Ki, double Kd);
  void TunePID();
  void TuneParameter(int param_num);
  void EnableTuning();
  void DisableTuning();
  bool IsTuningOn();
private:
  /*
  * Coefficients
  */ 
  std::vector<double> PID_;
  std::vector<double> PID_tuning_;
  std::vector<double> PID_error_;
  std::vector<bool> PID_adjusted_;
  // double Kp_;
  // double Ki_;
  // double Kd_;
  
  // /*
  // * Errors
  // */
  // double p_error_;
  // double i_error_;
  // double d_error_;

  /*
  * Tuning parameters
  */
  bool tuning_;
  int count_;
  // double p_tuning_;
  // double i_tuning_;
  // double d_tuning_;
  double best_error_;
  double current_error_;
};

#endif /* PID_H */
