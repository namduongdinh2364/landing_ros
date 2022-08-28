#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

// Constructor
PID::PID( double cmax, double cmin, double ckp, double cki, double ckd ) :
    max(cmax), min(cmin), kp(ckp), kd(ckd), ki(cki), pre_error(0), integral(0), pre_integral(0) {}

// Destructor
PID::~PID() {}

//Calculation of PID values Setpoint
double PID::calculate( double setpoint, double pv, double cdt )
{
    
    // Calculate error
    double error = setpoint - pv;
    // Proportional term
    double Pout = kp * error;
    // Integral term
    integral = pre_integral + 0.5 * (pre_error + error) * 0.1; // cdt is equals to 0.1 to avoid overflow
    double Iout = ki * integral; // Integral output
    // Derivative term
    double derivative = (error - pre_error) / 0.1; // cdt is equals to 0.1 to avoid overflow
    double Dout = kp * kd * derivative; // Derivative output
    // Calculate total output
    // std::cout << "Out P: " << Pout << endl;
    // std::cout << "Out I: " << Iout << endl;
    // std::cout << "Out D: " << Dout << endl;
    // std::cout << "===================="<< endl;
    double output = Pout + Iout + Dout;

    // Save error to previous error and previous integral value
    pre_error = error;
    pre_integral = integral;

    // Limit the max and min output
    if( output > max )
    {
        output = max;
    }

    else if( output < min )
    {
        output = min;
    }

    return output;
}


