#include "ros/ros.h"
#include <cstdlib>
#include <Eigen/Dense>
using namespace std;

string mavPos = " ";
string mavVel = " ";
string mavAtt = " ";
string Thrust = " 0 0 0 ";
string state = " ";
string mavAcc = " ";
string battery = " ";
string gpsStatus =" ";
string green   ="\033[;32m";
string red     ="\033[1;31m";
string yellow  ="\033[1;33m";
string blue    ="\033[;34m";
string normal  ="\033[0m";
string purple= "\033[0;35m" ;    
string cyan=   "\033[0;36m"  ;  
string prefix = " ";
string suffix = " ";
string prex = "", prey ="" , prez= "";

Eigen::Vector3d cv2eigen3d(cv::Mat m){
    Eigen::Vector3d c2e3 ; c2e3 << m.at<double>(0),m.at<double>(1) ,m.at<double>(2) ;
    return c2e3;
}

Eigen::Matrix3d cv2eigen9d(cv::Mat m){
    Eigen::Matrix3d c2e9 ; c2e9 << m.at<double>(0),m.at<double>(3) ,m.at<double>(6),m.at<double>(1),m.at<double>(4),m.at<double>(7),m.at<double>(2),m.at<double>(5),m.at<double>(8) ;
    return c2e9;
}