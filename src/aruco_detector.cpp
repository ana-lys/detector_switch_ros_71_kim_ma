#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "psycho_tarhs.h"
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace sensor_msgs;
using namespace cv;

class ArucoDetector
{
private:
	ros::NodeHandle nh;
	ros::Subscriber cam_info_sub,odom_sub;
	ros::Publisher pose_pub;
	std::string reference_frame;
	image_transport::Publisher image_pub;
	image_transport::Subscriber image_sub;
	image_transport::ImageTransport it;
    Eigen::Quaterniond base_quaternion;
	Eigen::Vector3d base_pose;
	cv::Mat inImage;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	std::vector<std::vector<cv::Point3f>> objPoints;
	std::vector<int> board_ids;
	cv::Ptr<cv::aruco::Board> board;
	cv::Ptr<cv::aruco::Dictionary> dictionary;
	bool cam_info_received;
	double trust_coeff;
public:
	ArucoDetector() : nh("~"), it(nh)
	{
		image_sub = it.subscribe("/camera/color/image_raw", 1, &ArucoDetector::image_callback, this);
		cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, &ArucoDetector::cam_info_callback, this);
		odom_sub = nh.subscribe("/mavros/local_position/odom", 1, &ArucoDetector::odom_callback, this);
		image_pub = it.advertise("result", 1);
		pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
		nh.param<std::string>("reference_frame", reference_frame, "aruco");

		cameraMatrix = Mat(3, 3, CV_64FC1);
		distCoeffs = Mat(1, 5, CV_64F);
		int i;
		for (i = 0; i < 5; i++)
		{
			distCoeffs.at<double>(0,i) = 0.0;
		}
		vector<Point3f> points_id3, points_id4, points_id5, points_id6;
		// points ID 3
		points_id3.push_back(Point3f(-0.125, 0.4, 0.0));
		points_id3.push_back(Point3f(0.125, 0.4, 0.0));
		points_id3.push_back(Point3f(0.125, 0.15, 0.0));
		points_id3.push_back(Point3f(-0.125, 0.15, 0.0));
		// points ID 4
		points_id4.push_back(Point3f(0.15, 0.125, 0.0));
		points_id4.push_back(Point3f(0.4, 0.125, 0.0));
		points_id4.push_back(Point3f(0.4, -0.125, 0.0));
		points_id4.push_back(Point3f(0.15, -0.125, 0.0));
		// points ID 5
		points_id5.push_back(Point3f(-0.125, -0.15, 0.0));
		points_id5.push_back(Point3f(0.125, -0.15, 0.0));
		points_id5.push_back(Point3f(0.125, -0.4, 0.0));
		points_id5.push_back(Point3f(-0.125, -0.4, 0.0));
		// points ID 6
		points_id6.push_back(Point3f(-0.4, 0.125, 0.0));
		points_id6.push_back(Point3f(-0.15, 0.125, 0.0));
		points_id6.push_back(Point3f(-0.15, -0.125, 0.0));
		points_id6.push_back(Point3f(-0.4, -0.125, 0.0));

		objPoints.push_back(points_id3);
		objPoints.push_back(points_id4);
		objPoints.push_back(points_id5);
		objPoints.push_back(points_id6);

		// std::cout << objPoints.at(3) << std::endl;
		// store ids are used
		board_ids.push_back(3);
		board_ids.push_back(4);
		board_ids.push_back(5);
		board_ids.push_back(6);
		// std::cout << "number objec: " << objPoints.size()<< std::endl;
		// std::cout << "number ids: " << board_ids.size()<< std::endl;
		dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
		board = cv::aruco::Board::create(objPoints, dictionary, board_ids);
	}

	void image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		if (cam_info_received)
		{  
			ros::Time curr_stamp = msg->header.stamp;
			cv_bridge::CvImagePtr cv_ptr;
			try
			{   
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				inImage = cv_ptr->image;
                
				std::vector<std::vector<cv::Point2f>> markerCorners;
				std::vector<int> markerIds;
				cv::aruco::detectMarkers(inImage, dictionary, markerCorners, markerIds);
				// if at least one marker detected
				if(markerIds.size() > 0) {
					// ROS_INFO_STREAM("trying");
					cv::Mat rvec, tvec;
					int valid = cv::aruco::estimatePoseBoard(markerCorners,
															 markerIds,
															 board, 
															 cameraMatrix, 
															 distCoeffs, 
															 rvec, 
															 tvec);
					if (valid) {
						Eigen::Vector3d Etv = cv2eigen3d(tvec);
						Eigen::Quaterniond Base_camera = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0.70710678118,-0.70710678118,0.0)));
						Eigen::Vector3d M_BQ  = Base_camera * Etv;
						Eigen::Vector3d M_IQ  = base_quaternion * M_BQ;
						geometry_msgs::PoseStamped poseMsg;
						poseMsg.header.frame_id = reference_frame;
						poseMsg.header.stamp = curr_stamp;
						poseMsg.pose.position.x = M_IQ(0);
						poseMsg.pose.position.y = M_IQ(1);
						poseMsg.pose.position.z = M_IQ(2);	
						poseMsg.pose.orientation.x = base_pose(0);
						poseMsg.pose.orientation.y = base_pose(1);
						poseMsg.pose.orientation.z = base_pose(2);
						poseMsg.pose.orientation.w = trust_coeff;
						pose_pub.publish(poseMsg);
					}
					
					
					if (image_pub.getNumSubscribers() > 0)
					{
						// show input with augmented information
						cv_bridge::CvImage out_msg;
						out_msg.header.stamp = curr_stamp;
						out_msg.encoding = sensor_msgs::image_encodings::RGB8;
						out_msg.image = inImage;
						image_pub.publish(out_msg.toImageMsg());
					}
				}
			}
			catch (cv_bridge::Exception& e)
			{
				// ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
		}
	}

	// wait for one camerainfo, then shut down that subscriber
	void cam_info_callback(const sensor_msgs::CameraInfo &msg)
	{
		cameraMatrix.at<double>(0, 0) = msg.K[0];
		cameraMatrix.at<double>(0, 1) = msg.K[1];
		cameraMatrix.at<double>(0, 2) = msg.K[2];
		cameraMatrix.at<double>(1, 0) = msg.K[3];
		cameraMatrix.at<double>(1, 1) = msg.K[4];
		cameraMatrix.at<double>(1, 2) = msg.K[5];
		cameraMatrix.at<double>(2, 0) = msg.K[6];
		cameraMatrix.at<double>(2, 1) = msg.K[7];
		cameraMatrix.at<double>(2, 2) = msg.K[8];
		cam_info_received = true;
		std::cout << "Get cameraMaxtrix done!!!" << std::endl;
		cam_info_sub.shutdown();
	}
    
	void odom_callback(const nav_msgs::Odometry &msg){
		base_quaternion.w() = msg.pose.pose.orientation.w;
		base_quaternion.x() = msg.pose.pose.orientation.x;
		base_quaternion.y() = msg.pose.pose.orientation.y;
		base_quaternion.z() = msg.pose.pose.orientation.z;
		base_pose = toEigen(msg.pose.pose.position);
		Eigen::Vector3d linear_velocity = toEigen(msg.twist.twist.linear);
		Eigen::Vector3d angular_velocity = toEigen(msg.twist.twist.angular);
		trust_coeff = 0.05/(log(linear_velocity.squaredNorm()+1.5)*log(angular_velocity.squaredNorm()+1.135));
	}
	
	inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
     Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
     return ev3;
    }
	inline Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {
     Eigen::Vector3d ev3(p.x, p.y, p.z);
     return ev3;
    }

	
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_detector");

  ArucoDetector node;

  ros::spin();
}
