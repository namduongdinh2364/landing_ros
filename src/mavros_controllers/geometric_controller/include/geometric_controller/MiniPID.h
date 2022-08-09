#ifndef MINIPID_H
#define MINIPID_H

class MiniPID{
public:
    MiniPID(double, double, double);
    MiniPID(double, double, double, double);
    void setP(double);
    void setI(double);
    void setD(double);
    void setF(double);
    void setPID(double, double, double);
    void setPID(double, double, double, double);
    void setMaxIOutput(double);
    void setOutputLimits(double);
    void setOutputLimits(double,double);
    void setDirection(bool);
    void setSetpoint(double);
    void reset();
    void setOutputRampRate(double);
    void setSetpointRange(double);
    void setOutputFilter(double);
    double getOutput();
    double getOutput(double);
    double getOutput(double, double);

private:
    double clamp(double, double, double);
    bool bounded(double, double, double);
    void checkSigns();
    void init();
    double P;
    double I;
    double D;
    double F;

    double maxIOutput;
    double maxError;
    double errorSum;

    double maxOutput; 
    double minOutput;

    double setpoint;

    double lastActual;

    bool firstRun;
    bool reversed;

    double outputRampRate;
    double lastOutput;

    double outputFilter;

    double setpointRange;
};
#endif
// /// \file  pid_controller_base.h
// ///   \brief Contains the definition of the MiniPID class.
// ///

// // #ifndef PID_CONTROLLER_BASE_H
// // #define PID_CONTROLLER_BASE_H

// #include <limits>
// #include <iostream>
// #include <time.h>
// #include <ros/ros.h>
// #include <eigen3/Eigen/Dense>
// /// \brif A class that implements PID control algorithm.
// ///
// /// A class that implements a Proportional-Integral-Derivative (PID) algorithm
// /// used to control dynamical systems. Algorithm is computed in its discrete form:
// /// u = kp * e + ui_old + ki * e * td + kd * (e - e_old) / td, with:
// /// u - output of the algorithm
// /// kp - proportional gain
// /// ki - integral gain
// /// kd - derivative gain
// /// ui_old - integral value from the previous step
// /// e - error of the measured value w.r.t. the reference
// /// e_old - error of the measured value from the previous step
// /// td - time elapsed from the previous step (in seconds)
// ///
// class MiniPID
// {
// public:

//     /// \brief PID controller default constructor.
//     ///
//     /// Initializes PID gains to zero.
//     ///
//     MiniPID();

//     /// PID controller contructor
//     ///
//     /// \param kp Proportional gain.
//     /// \param ki Integral gain.
//     /// \param kd Derivative gain.
//     ///
//     /// Initializes PID gains to the given values.
//     ///
//     MiniPID(double kp, double ki, double kd);

//     /// \brief PID controller destructor.
//     ///
//     /// Does nothing.
//     ///
//     ~MiniPID();

//     /// Returns the current proportional gain.
//     ///
//     double getKp(void);

//     /// Returns the current integral gain.
//     ///
//     double getKi(void);

//     /// Returns the current derivative gain.
//     ///
//     double getKd(void);

//     /// Returns the current maximal control value (upper saturation limit).
//     ///
//     double getUMax(void);

//     /// Returns the current minimal control value (lower saturation limit).
//     ///
//     double getUMin(void);

//     /// Returns the current maximal allowable time step.
//     ///
//     double getTdMax(void);

//     /// Returns the current minimal allowable time step.
//     ///
//     double getTdMin(void);

//     /// Sets proportional gain
//     /// \param kp The desired value of the proportional gain
//     ///
//     void setKp(double kp);

//     /// Sets integral gain
//     /// \param ki The desired value of the integral gain
//     ///
//     void setKi(double ki);

//     /// Sets integral gain
//     /// \param kd The desired value of the derivative gain
//     ///
//     void setKd(double kd);

//     /// Sets maximal control value (upper saturation limit).
//     /// \param u_max The desired maximal control value.
//     ///
//     void setUMax(double u_max);

//     /// Sets minimal control value (lower saturation limit).
//     /// \param u_min The desired minimal control value.
//     ///
//     void setUMin(double u_min);

//     /// Sets maximal allowable time step.
//     /// \param td_max The desired maximal time step.
//     ///
//     void setTdMax(double td_max);

//     /// Sets minimal allowable time step.
//     /// \param td_min The desired minimal time step.
//     ///
//     void setTdMin(double td_min);

//     /// Computes PID algorithm.
//     /// \param ref Current referent value.
//     /// \param meas Current measured value.
//     /// \return Output of the PID algorithm (control value).
//     ///
//     double compute(double ref, double meas, ros::Time last_time);
//     // Eigen::Vector3d compute_linvel_effort(Eigen::Vector3d goal, Eigen::Vector3d current, ros::Time last_time);
//     // double compute_vel(double error, ros::Duration dt);
//     // double bound(double output, double max, double min);
// // protected:
// private:

//     // Proportional gain.
//     double kp_;

//     // Integral gain.
//     double ki_;

//     // Derivative gain.
//     double kd_;

//     // The value of the integral term from the previous step.
//     double ui_old_;

//     // Error value from the previous step.
//     double error_old_;

//     // maximal control value (upper saturation limit)
//     double u_max_;

//     // minimal control value (lower saturation limit)
//     double u_min_;

//     // maximal allowable time step
//     double td_max_;

//     // minimal allowable time step
//     double td_min_;

//     // flag indicates if the algorithm is executed for the first time
//     bool is_first_pass_;

//     // time of the previous step
//     timespec time_old_;

// };

// // #endif // PID_CONTROLLER_BASE_H