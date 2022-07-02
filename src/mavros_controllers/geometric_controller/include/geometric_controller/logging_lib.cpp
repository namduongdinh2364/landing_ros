#include <iostream>
#include <fstream>
#include <ros/ros.h>
using namespace std;
string name="";
void creates()
{  
	std::fstream file; 
    name ="/home/analys/log csv/long_cv2.csv";
	file.open(name, std::ios::out | std::ios::app); 
	file << "TimeStamp,errorX,errorY,errorZ,feedbackX,feedbackY,feedbackZ,ImuX,ImuY,ImuZ,setAX,setAY,setAZ,setVX,setVY,setVZ,odoVX,odoVY,odoVZ,Thrust\n";
    file.close(); 
} 
 
void updates(double time, 
                   double errorX, double errorY, double errorZ, 
                   double feedbackX, double feedbackY, double feedbackZ, 
                   double ImuX, double ImuY, double ImuZ, 
                   double setAX, double setAY, double setAZ, 
                   double setVX, double setVY, double setVZ,
                   double odoVX, double odoVY, double odoVZ,
                   double thrust
                   )

{   int time_=floor(time*1000);
        if(time_%100 < 8 ){
	std::fstream file;  
    file.open(name, std::ios::out | std::ios::app); 
    if(!file.is_open()) cout << "file oppen error";
    file <<setprecision(2)<<fixed<< time_/1000.0 << ", " 
                         << std::fixed << std::setprecision(8) << errorX << ", " 
						 << std::fixed << std::setprecision(8) << errorY << ", " 
						 << std::fixed << std::setprecision(8) << errorZ << ", "
                         << std::fixed << std::setprecision(8) << feedbackX << ", " 
						 << std::fixed << std::setprecision(8) << feedbackY << ", " 
						 << std::fixed << std::setprecision(8) << feedbackZ << ", "
						 << std::fixed << std::setprecision(8) << ImuX << ", " 
						 << std::fixed << std::setprecision(8) << ImuY << ", " 
						 << std::fixed << std::setprecision(8) << ImuZ << ", "
                         << std::fixed << std::setprecision(8) << setAX << ", " 
						 << std::fixed << std::setprecision(8) << setAY << ", " 
						 << std::fixed << std::setprecision(8) << setAZ << ", "
                         << std::fixed << std::setprecision(8) << setVX << ", " 
						 << std::fixed << std::setprecision(8) << setVY << ", " 
						 << std::fixed << std::setprecision(8) << setVZ << ", "
                         << std::fixed << std::setprecision(8) << odoVX << ", " 
						 << std::fixed << std::setprecision(8) << odoVY << ", " 
                         << std::fixed << std::setprecision(8) << odoVZ << ", " 
						 << std::fixed << std::setprecision(8) << thrust<< "\n";

	file.close(); 
} 
}
