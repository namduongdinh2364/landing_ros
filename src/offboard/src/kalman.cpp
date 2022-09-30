#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include "pid_controller/States.h"

# define M_PI 3.14159265358979323846  /* pi */

using namespace Eigen;

class Kalman
{
    private:
        // Private class attributes
        ros::NodeHandle po_nh;
        ros::Subscriber sub;
        ros::Publisher pub;

        MatrixXd T; // Posteriori estimate covariance matrix
        MatrixXd Q; // Covariance of the process noise
        MatrixXd R; // Covariance of the observation noise
        MatrixXd A; // State transition matrix
        MatrixXd H; // Observation model
        MatrixXd X; // State vector
        MatrixXd Z; // Innovation vectot
        MatrixXd S1; // Covariance of the innovation
        MatrixXd Kg; // Kalman gain
        float dt; // Delta of time
        int first_iter; // variable to check the first iteration of the algorithm

    public:
        // Public class attributes and methods
        Kalman(ros::NodeHandle ao_nh) : po_nh( ao_nh ), first_iter(0), dt(1), T(4,4), Q(10,10), R(4,4), 
                                        A(4,4), H(4,4), X(4,1), Z(4,1), S1(4,4), Kg(4,4)
        {
            // Publisher type object_detector::States, it publishes in /predicted_states topic
            pub = po_nh.advertise<pid_controller::States>( "/predicted_states", 10 ) ;
            // Subscriber to /states topic from object_detector/States
            sub = po_nh.subscribe("/kalman_states", 10, &Kalman::predictionsDetectedCallback, this); 
            // Delta of time for the transition matrix
            this->dt = 0.1;
            this->first_iter = 0;


            // Posteriori estimate covariance matrix initialization
            this->T <<  2.0, 0.0, 0.0, 0.0, 
                        0.0, 1e-3, 0.0, 0.0, 
                        0.0, 0.0, 2.0, 0.0,
                        0.0, 0.0, 0.0, 1e-3;

            // Covariance of the process noise initialization
            this->Q = 1e-4*MatrixXd::Identity(4,4);
            // Covariance of the observation noise initialization
            this->R = 1e-2*MatrixXd::Identity(4,4);
            this->R(0, 0) = 1e-4;
            this->R(3, 3) = 1e-4;

            // State vector initialization
            this->X = MatrixXd::Zero(4,1); 
            // Innovation vectot initialization
            this->Z = MatrixXd::Zero(4,1); 
            // Covariance of the innovation initialization
            this->S1 = MatrixXd::Zero(4,4); 
            // Kalman gain initialization
            this->Kg = MatrixXd::Zero(4,4); 

            // State transition matrix initialization
            this->A << 1.0, dt, 0.0, 0.0, 
                        0.0, 1.0, 0.0, 0.0, 
                        0.0, 0.0, 1.0, dt,
                        0.0, 0.0, 0.0, 1.0;

            // Observation model initialization
            this->H <<  1.0 ,0.0 ,0.0 ,0.0, 
                        0.0 ,1.0 ,0.0 ,0.0, 
                        0.0 ,0.0 ,1.0 ,0.0, 
                        0.0 ,0.0 ,0.0 ,1.0;

        }

        // Subscriber callback
        void predictionsDetectedCallback(const pid_controller::States& msg)
        {	
            // Creation of a States object to publish the info
            pid_controller::States predictions;

            MatrixXd measurement(4,1); // Vector to store the observations
            measurement = MatrixXd::Zero(4,1);

            // If it is the first iteration, initialize the states with the first observation
            if(this->first_iter==0)
            {
                this->X(0,0) = msg.Xm; // x
                this->X(1,0) = msg.Ym; // y
                this->X(2,0) = 0; // x'
                this->X(3,0) = 0; // y'
                this->first_iter = 1;
            }

            // Prediction step
            this->X = this->A*this->X; 
            this->T = ((this->A*this->T)*this->A.transpose())+this->Q;

            // Update step, assign the observations to the measurement vector 
            measurement(0,0) = msg.Xm;
            measurement(1,0) = msg.Ym;
            measurement(2,0) = msg.Vx;
            measurement(3,0) = msg.Vy;

            // If there is a valid measurement (the detector found the coordinates)
            // if((measurement(0)>0) && (measurement(1)>0) && (measurement(2)>0) && (measurement(3)>0)) 
            // {
                // Update step
                this->Z = this->H * this->X;

                this->S1 = ((this->H*this->T)*this->H.transpose()) + R; 
                this->Kg = (this->T*this->H.transpose())*this->S1.inverse();
                this->X = this->X + this->Kg*(measurement - this->Z); 
                this->T = this->T - ((this->Kg*this->S1)*this->Kg.transpose()); 
            // }

            // Assign the predictions to the publisher object
            predictions.Xm = this->X(0,0);
            predictions.Ym = this->X(1,0);
            predictions.Vx = this->X(2,0);
            predictions.Vy = this->X(3,0);
            std::cout << "===" << predictions << std::endl;
            pub.publish(predictions);
        }
};


int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "kalman"); 
    ros::NodeHandle n;
    Kalman kf(n);
    ros::spin();

    return 0;
}   
