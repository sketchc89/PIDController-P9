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

void PID::TuneParameter(int param_num){
    int tune_p = 0, tune_i = 0, tune_d = 0;

    if (param_num == 0) {
        tune_p = 1;
    } else if (param_num == 1) {
        tune_i = 1;
    } else if (param_num == 2) {
        tune_d = 1;
    } else {
        throw "Invalid tuning parameter";
    }
    
    UpdatePID(PID_[0] + tune_p*PID_tuning_[0], 
            PID_[1] + tune_i*PID_tuning_[1], 
            PID_[2] + tune_d*PID_tuning_[2]);
    current_error_ = TotalError();
    if (fabs(current_error_) < fabs(best_error_)) {
        std::cout << "Increasing param" << param_num << "\n";
        best_error_ = current_error_;
        PID_tuning_[param_num] *= 1.1;
    } else {
        UpdatePID(PID_[0] + tune_p*PID_tuning_[0], 
                PID_[1] + tune_i*PID_tuning_[1], 
                PID_[2] + tune_d*PID_tuning_[2]);
        current_error_ = TotalError();
        if (fabs(current_error_) < fabs(best_error_)) {
            std::cout << "Decreasing param" << param_num << "\n";
            best_error_ = current_error_;
            PID_tuning_[param_num] *= 1.1;
        } else {
            std::cout << "Reducing tuning parameter " << param_num << "\n";
            UpdatePID(PID_[0] + tune_p*PID_tuning_[0], 
                    PID_[1] + tune_i*PID_tuning_[1], 
                    PID_[2] + tune_d*PID_tuning_[2]);
            PID_tuning_[param_num] *= 0.9;
        }
    }
}

void PID::TunePID() {
    //TODO: Implement twiddle
    //PID::p_tuning;
    
    std::cout << "\nCount: " << count_;
    // PID_tuning_[0] + PID_tuning_[1] + PID_tuning_[2] > 0.001 && 
    count_++;
    
    current_error_ += fabs(TotalError());

    std::cout << "\nCount: " << count_ << "\tError: " << current_error_ << "\tBest Error: " << best_error_;
    if (count_ == 750) {
        current_error_ = 0;
    } else if (count_ == 2250) {
        if (current_error_ < best_error_){
            best_error_ = current_error_;
        }
        current_error_ = 0;
        UpdatePID(PID_[0] + PID_tuning_[0], PID_[1], PID_[2]); //p up
    } else if (count_ == 3750) {
        if (current_error_ < best_error_){
            best_error_ = current_error_;
            PID_adjusted_[0] = true;
        } else {
            UpdatePID(PID_[0] - PID_tuning_[0], PID_[1], PID_[2]);
        }
        current_error_ = 0;
        UpdatePID(PID_[0] - PID_tuning_[0], PID_[1], PID_[2]); //p down
    } else if (count_ == 5250) {
        if (current_error_ < best_error_){
            best_error_ = current_error_;
            PID_adjusted_[0] = true;
        } else if (PID_adjusted_[0]) {
            UpdatePID(PID_[0] + 2*PID_tuning_[0], PID_[1], PID_[2]);
        } else {
            UpdatePID(PID_[0] + PID_tuning_[0], PID_[1], PID_[2]);
        }
        
        if (!PID_adjusted_[0]) {
            PID_tuning_[0] *= 0.9;
        }
        current_error_ = 0;
        UpdatePID(PID_[0], PID_[1] + PID_tuning_[1], PID_[2]); //i up
    } 
    else if (count_ == 6750) {
        if (current_error_ < best_error_){
            best_error_ = current_error_;
            PID_adjusted_[1] = true;
        } else {
            UpdatePID(PID_[0], PID_[1] - PID_tuning_[1], PID_[2]);
        }
        current_error_ = 0;
        UpdatePID(PID_[0], PID_[1] - PID_tuning_[1], PID_[2]); //i down
    } else if (count_ == 8250) {
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
        }
        current_error_ = 0;
        UpdatePID(PID_[0],  PID_[1], PID_[2] + PID_tuning_[2]); // d up
    } else if (count_ == 9750) {
        if (current_error_ < best_error_){
            best_error_ = current_error_;
            PID_adjusted_[2] = true;
        } else {
            UpdatePID(PID_[0],  PID_[1], PID_[2] - PID_tuning_[2]);
        }
        current_error_ = 0;
        UpdatePID(PID_[0], PID_[1], PID_[2] - PID_tuning_[2]); // d down
    } else if (count_ == 11250) {
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
