#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

using namespace std;


class Locomotion_Driver{
	private: 
		ros::Publisher * command_vel_pub;
		ros::Subscriber aruco_sub;
		ros::Subscriber angle_sub;
		void fiducialsCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr & msg);
		void angleCallback(const std_msgs::Float64 & msg);
		
	public:
		Locomotion_Driver(ros::NodeHandle &nh);
};


//building this to keep running until aruco method can no longer be seen. 
//Then will spin the rest of the degree needed

void Locomotion_Driver::fiducialsCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
	try {
		double angle_read;
		double roll, pitch, yaw;

		for (size_t i=0; i<msg->transforms.size(); i++){
			const fiducial_msgs::FiducialTransform &ft = msg->transforms[i];
	
			tf2::Vector3 Vec3(ft.transform.translation.x, ft.transform.translation.y, ft.transform.translation.z);
		
			tf2::Quaternion q(ft.transform.rotation.x,
    				 ft.transform.rotation.y,
	                	 ft.transform.rotation.z,
        	        	 ft.transform.rotation.w);

//			ROS_INFO("Roll %d Pitch %d Yaw %d" , q.x(), q.y(), q.z());
		}	
	}
	catch(...){
	   ROS_INFO("Default Exception\n");
	}
	
}

void Locomotion_Driver::angleCallback(const std_msgs::Float64 &msgs){
	try{
	    ROS_INFO("test %d", msgs);	

	}

	catch(...){
	   ROS_INFO("Default Exception\n");
	}
}

Locomotion_Driver::Locomotion_Driver(ros::NodeHandle & nh) {

	aruco_sub = nh.subscribe("/fiducial_transforms", 1, 
			&Locomotion_Driver::fiducialsCallback, this);
	
	angle_sub = nh.subscribe("/fiducial_angle", 1, &Locomotion_Driver::angleCallback, this);

	command_vel_pub = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));


}

int main(int argc, char ** argv) {
//	ROS_INFO("checked");
	ros::init(argc, argv, "locomotion_startup");
	ros::NodeHandle nh("~");
	
	Locomotion_Driver * node = new Locomotion_Driver(nh);
	ros::spin();
	
	return 0;
} 
