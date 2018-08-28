#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    PID::p_error = 0.0;
    PID::i_error = 0.0;
    PID::d_error = 0.0;

    PID::p_tuning = Kp*0.1;
    PID::i_tuning = Ki*0.1;
    PID::d_tuning = Kd*0.1;
}

void PID::UpdateError(double cte) {
    PID::d_error = cte - PID::p_error;
    PID::p_error = cte;
    PID::i_error += cte;
}

double PID::TotalError() {
    return -PID::p_error*PID::Kp - PID::i_error*PID::Ki - PID::d_error*Kd;
}

void PID::TunePID() {
    //TODO: Implement twiddle
    //PID::p_tuning;
}