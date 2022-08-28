#ifndef PID_H
#define PID_H

class PID
{
    private:
        double max; // max - maximum output value
        double min; // min - minimum output value
        double kp; // Kp -  proportional gain
        double kd; // Kd -  derivative gain
        double ki; // Ki -  Integral gain
        double pre_error; // Error at (t-1)
        double integral; // Integral term
        double pre_integral; // Integral term at (t-1)

    public:
        // Class constructor
        PID(double cmax, double cmin, double ckp, double ckd, double cki);
        // Compute PID output
        double calculate( double setpoint, double pv, double cdt);
        // Class destructor
        ~PID();

    
};

#endif
