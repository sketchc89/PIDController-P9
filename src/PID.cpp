#include <iostream>
#include <cmath>
#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    
    UpdatePID(Kp, Ki, Kd);
    PID_error_ = {0.0, 0.0, 0.0};
    PID_tuning_ = {Kp*0.1, Ki*0.1, Kd*0.1};
    PID_adjusted_ = {false, false, false};
    count_ = 0;
    best_error_ = 10000000000000.0;
    current_error_ = 0;
    EnableTuning();
}

void PID::UpdateError(double cte) {
    PID_error_[2] = cte - PID_error_[0];
    PID_error_[0] = cte;
    PID_error_[1] += cte;
}

void PID::UpdatePID(double Kp, double Ki, double Kd) {
    PID_ = {Kp, Ki, Kd};
}

void PID::DisplayPID() {
    std::cout << "\nPID Parameters\nP: " << PID_[0] << "\tI: " << PID_[1] << "\tD: " << PID_[2];
    std::cout << "\nTuning Parameters\nP: " << PID_tuning_[0] << "\tI: " << PID_tuning_[1] << "\tD: " << PID_tuning_[2] << "\n";
}

double PID::TotalError() {
    return -PID_error_[0]*PID_[0] - PID_error_[1]*PID_[1] - PID_error_[2]*PID_[2];
}

void PID::TunePID() {
    //TODO: Implement twiddle
    //PID::p_tuning;
    
    count_++;
    current_error_ += fabs(TotalError());

    std::cout << "\nCount: " << count_ << "\tError: " << current_error_ << "\tBest Error: " << best_error_;
    if (count_ == 570) {
        current_error_ = 0;
    
    } else if (count_ == 1670) {
        if (current_error_ < best_error_){
            best_error_ = current_error_;
        }
        current_error_ = 0;
        UpdatePID(PID_[0] + PID_tuning_[0], PID_[1], PID_[2]); //p up
    
    } else if (count_ == 2770) {
        if (current_error_ < best_error_){
            best_error_ = current_error_;
            PID_adjusted_[0] = true;
            PID_tuning_[0] *= 1.1;
        } 
        current_error_ = 0;
        UpdatePID(PID_[0] - 2*PID_tuning_[0], PID_[1], PID_[2]); //p down
    
    } else if (count_ == 3870) {
        if (current_error_ < best_error_){
            best_error_ = current_error_;
            PID_adjusted_[0] = true;
            PID_tuning_[0] *= 1.1;
        } else if (PID_adjusted_[0]) {
            UpdatePID(PID_[0] + 2*PID_tuning_[0], PID_[1], PID_[2]);
        } else {
            UpdatePID(PID_[0] + PID_tuning_[0], PID_[1], PID_[2]);
        }
        
        if (!PID_adjusted_[0]) {
            PID_tuning_[0] *= 0.9;
        } else {
            PID_tuning_[0] *= 1.1;
        }
        current_error_ = 0;
        UpdatePID(PID_[0], PID_[1] + PID_tuning_[1], PID_[2]); //i up
    } 
    
    else if (count_ == 4970) {
        if (current_error_ < best_error_){
            best_error_ = current_error_;
            PID_adjusted_[1] = true;
        }
        current_error_ = 0;
        UpdatePID(PID_[0], PID_[1] - 2*PID_tuning_[1], PID_[2]); //i down
    
    } else if (count_ == 6070) {
        if (current_error_ < best_error_){
            best_error_ = current_error_;
            PID_adjusted_[1] = true;
        } else if (PID_adjusted_[1]) {
            UpdatePID(PID_[0], PID_[1] + 2*PID_tuning_[1], PID_[2]);
        } else {
            UpdatePID(PID_[0], PID_[1] + PID_tuning_[1], PID_[2]);
        }
        
        if (!PID_adjusted_[1]) {
            PID_tuning_[1] *= 0.9;
        } else {
            PID_tuning_[1] *= 1.1;
        }
        current_error_ = 0;
        UpdatePID(PID_[0],  PID_[1], PID_[2] + PID_tuning_[2]); // d up
    
    } else if (count_ == 7170) {
        if (current_error_ < best_error_){
            best_error_ = current_error_;
            PID_adjusted_[2] = true;
        } 
        current_error_ = 0;
        UpdatePID(PID_[0], PID_[1], PID_[2] - 2*PID_tuning_[2]); // d down
    
    } else if (count_ == 8270) {
        if (current_error_ < best_error_){
            best_error_ = current_error_;
            PID_adjusted_[2] = true;
        } else if (PID_adjusted_[2]) {
            UpdatePID(PID_[0], PID_[1], PID_[2] + 2*PID_tuning_[2]);
        } else {
            UpdatePID(PID_[0], PID_[1], PID_[2] + PID_tuning_[2]);
        }

        if (!PID_adjusted_[2]) {
            PID_tuning_[2] *= 0.9;
        } else {
            PID_tuning_[2] *= 1.1;
        }
        current_error_ = 0;
        count_ = 0;
        PID_adjusted_ = {false, false, false};
    }      
}

void PID::EnableTuning(){
    tuning_ = true;
}

void PID::DisableTuning(){
    tuning_ = false;
}

bool PID::IsTuningOn(){
    return tuning_;
}
