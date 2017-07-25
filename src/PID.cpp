#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp0, double Ki0, double Kd0) {
    // initialize coefficients
    Kp = Kp0;
    Ki = Ki0;
    Kd = Kd0;

    //initialize error values
    p_error = 0.;
    i_error = 0.;
    d_error = 0.;

}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
    return Kp * p_error + Ki * i_error + Kd * d_error;
}

