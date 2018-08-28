#include <iostream>
#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    p_error_ = 0.0;
    i_error_ = 0.0;
    d_error_ = 0.0;

    p_tuning_ = Kp_*0.1;
    i_tuning_ = Ki_*0.1;
    d_tuning_ = Kd_*0.1;

    EnableTuning();
}

void PID::UpdateError(double cte) {
    d_error_ = cte - p_error_;
    p_error_ = cte;
    i_error_ += cte;
}

double PID::TotalError() {
    return -p_error_*Kp_ - i_error_*Ki_ - d_error_*Kd_;
}

void PID::TunePID() {
    //TODO: Implement twiddle
    //PID::p_tuning;
    if (p_tuning_ + i_tuning_ + d_tuning_ > 0.001) {
        std::cout << "\nTuning";
        Kp_ = Kp_;
        Ki_ = Ki_;
        Kd_ = Kd_;
    } else {
        std::cout << "\nAlready tuned";
    }
}

void PID::EnableTuning(){
    tuning_ = true;
}

void PID::DisableTuning(){
    tuning_ = false;
}