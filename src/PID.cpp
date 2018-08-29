#include <iostream>
#include <cmath>
#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    
    UpdatePID(Kp, Ki, Kd);
    PID_error_ = {0.0, 0.0, 0.0};
    PID_tuning_ = {Kp*0.01, Ki*0.01, Kd*0.01};
    PID_adjusted_ = {false, false, false};
    count_ = 0; 
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
    double current_error;
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
    current_error = TotalError();
    if (fabs(current_error) < fabs(best_error_)) {
        std::cout << "Increasing param" << param_num << "\n";
        best_error_ = current_error;
        PID_tuning_[param_num] *= 1.1;
    } else {
        UpdatePID(PID_[0] + tune_p*PID_tuning_[0], 
                PID_[1] + tune_i*PID_tuning_[1], 
                PID_[2] + tune_d*PID_tuning_[2]);
        current_error = TotalError();
        if (fabs(current_error) < fabs(best_error_)) {
            std::cout << "Decreasing param" << param_num << "\n";
            best_error_ = current_error;
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
    best_error_ = 10000000000000.0;
    
    std::cout << "\nCount: " << count_;
    // PID_tuning_[0] + PID_tuning_[1] + PID_tuning_[2] > 0.001 && 
    count_++;
    
    double current_error = 0;
    current_error += TotalError();

    if (count_ == 200) {
        current_error = 0;
    } else if (count_ == 1200) {
        if (current_error < best_error_){
            best_error_ = current_error;
        }
        current_error = 0;
        UpdatePID(PID_[0] + PID_tuning_[0], PID_[1], PID_[2]); //p up
    } else if (count_ == 2200) {
        if (current_error < best_error_){
            best_error_ = current_error;
            PID_adjusted_[0] = true;
        } else {
            UpdatePID(PID_[0] - PID_tuning_[0], PID_[1], PID_[2]);
        }
        current_error = 0;
        UpdatePID(PID_[0] - PID_tuning_[0], PID_[1], PID_[2]); //p down
    } else if (count_ == 3200) {
        if (current_error < best_error_){
            best_error_ = current_error;
            PID_adjusted_[0] = true;
        } else if (PID_adjusted_[0]) {
            UpdatePID(PID_[0] + 2*PID_tuning_[0], PID_[1], PID_[2]);
        } else {
            UpdatePID(PID_[0] + PID_tuning_[0], PID_[1], PID_[2]);
        }
        
        if (!PID_adjusted_[0]) {
            PID_tuning_[0] *= 0.9;
        }
        current_error = 0;
        UpdatePID(PID_[0], PID_[1] + PID_tuning_[1], PID_[2]); //i up
    } 
    else if (count_ == 4200) {
        if (current_error < best_error_){
            best_error_ = current_error;
            PID_adjusted_[1] = true;
        } else {
            UpdatePID(PID_[0], PID_[1] - PID_tuning_[1], PID_[2]);
        }
        current_error = 0;
        UpdatePID(PID_[0], PID_[1] - PID_tuning_[1], PID_[2]); //i down
    } else if (count_ == 5200) {
        if (current_error < best_error_){
            best_error_ = current_error;
            PID_adjusted_[1] = true;
        } else if (PID_adjusted_[1]) {
            UpdatePID(PID_[0], PID_[1] + 2*PID_tuning_[1], PID_[2]);
        } else {
            UpdatePID(PID_[0], PID_[1] + PID_tuning_[1], PID_[2]);
        }
        
        if (!PID_adjusted_[1]) {
            PID_tuning_[1] *= 0.9;
        }
        current_error = 0;
        UpdatePID(PID_[0],  PID_[1], PID_[2] + PID_tuning_[2]); // d up
    } else if (count_ == 6200) {
        if (current_error < best_error_){
            best_error_ = current_error;
            PID_adjusted_[2] = true;
        } else {
            UpdatePID(PID_[0],  PID_[1], PID_[2] - PID_tuning_[2]);
        }
        current_error = 0;
        UpdatePID(PID_[0], PID_[1], PID_[2] - PID_tuning_[2]); // d down
    } else if (count_ == 7200) {
        if (current_error < best_error_){
            best_error_ = current_error;
            PID_adjusted_[2] = true;
        } else if (PID_adjusted_[2]) {
            UpdatePID(PID_[0], PID_[1], PID_[2] + 2*PID_tuning_[2]);
        } else {
            UpdatePID(PID_[0], PID_[1], PID_[2] + PID_tuning_[2]);
        }

        if (!PID_adjusted_[2]) {
            PID_tuning_[2] *= 0.9;
        }
        current_error = 0;
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
