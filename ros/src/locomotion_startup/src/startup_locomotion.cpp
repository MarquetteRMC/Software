#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <math.h>

#define PI 3.14159265


using namespace std;
double angle_to_drive;
bool aruco_detected = false;

class Locomotion_Driver{
	private: 
		ros::Publisher * command_vel_pub;
		ros::Subscriber aruco_sub;
		ros::Subscriber angle_sub;
		ros::Subscriber Imu_sub;

		void fiducialsCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr & msg);
		void angleCallback(const std_msgs::Float64 & msg);
		void ImuCallback(const sensor_msgs::Imu::ConstPtr & msg);
		
	public:
		Locomotion_Driver(ros::NodeHandle &nh);
};


//building this to keep running until aruco method can no longer be seen. 
//Finds the angle between the center of the aruco marker and the rgb camera
void Locomotion_Driver::fiducialsCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
	try {
		double last_x;
                double last_y;
                double angle;
		int timeout_counter;
		int angular_speed;
		geometry_msgs::Twist twist;
		for (size_t i=0; i<msg->transforms.size(); i++){
			const fiducial_msgs::FiducialTransform &ft = msg->transforms[i];
	
			tf2::Vector3 Vec3(ft.transform.translation.x, ft.transform.translation.y, ft.transform.translation.z);
		
			tf2::Quaternion q(ft.transform.rotation.x,
    				 ft.transform.rotation.y,
	                	 ft.transform.rotation.z,
        	        	 ft.transform.rotation.w);

			ROS_INFO("rot x %f rot y %f rot z %f" , q.x(), q.y(), q.z());
			ROS_INFO("vec x %f vec y %f vec z %f" , Vec3.x(), Vec3.y(), Vec3.z());
			last_x = Vec3.x();
			last_y = Vec3.y();
		}
		//convert the x and y vectors into an angle. Subtract 45 since is straight ahead
		angle = (atan(last_x / last_y) * 180/PI) - 0;
		angle_to_drive = 90 - angle;
		ROS_INFO("The angle of the marker is %f degrees." , angle);
		//for first aruco marker. Prevents timeout_counter from stopping if marker isn't found right away
		if(angle > 0.0){
			aruco_detected = true;
		}
		//spin right away
                if(aruco_detected == false){
                        ROS_INFO("printing inital spin");
                        angular_speed == 22;
                        twist.angular.z = angular_speed;
                        command_vel_pub->publish(twist);
                }

		//check if aruco gets missed. Prevent it from jerking
		if(angle <= 0.0 && aruco_detected == true){
			timeout_counter = timeout_counter + 1;
		}
		//keep spinning 
		if(angle > 0.0 && timeout_counter <= 3){
			angular_speed = 22;
			twist.angular.z = angular_speed;
			command_vel_pub->publish(twist);
			timeout_counter = 0;
		}
		
	}
	catch(...){
	   ROS_INFO("Default Exception\n");
	}
	
}

void Locomotion_Driver::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
	try{
		/*ROS_INFO("angular velocity x %f, angular velocity y %f angular velocity z %f", 
		msg->angular_velocity.x,
		msg->angular_velocity.y,
		msg->angular_velocity.z);*/

		//assuming the anglular velocity to turn is on the yaw axis so z. determine in testing
		//publishing at a rate of 250 Hz 
		//angular_velocity = change of degree / change of time
		double angular_speed;
		double angular_velocity = msg->angular_velocity.z;
		double time_change = 1/250; //need to change this to the imu_pub hz value as it fluctuates around 250
		geometry_msgs::Twist twist;
		if (angle_to_drive > 0 && aruco_detected == true){
			double theta;
			theta = angular_velocity * time_change;
			angle_to_drive = angle_to_drive - theta;
			angular_speed = 22;
			twist.angular.z = angular_speed;
			command_vel_pub->publish(twist);
		}
		//drive forward
		else if(angle_to_drive < 0 && aruco_detected == true){
			int drive_forward;
			drive_forward = 30;
			twist.linear.x = drive_forward;
			command_vel_pub->publish(twist);
		}
		ROS_INFO("angle to drive %f", angle_to_drive);
	}
	
	catch(...){
	    ROS_INFO("Default Exception\n");
	}

}

void Locomotion_Driver::angleCallback(const std_msgs::Float64 &msgs){
	try{
	    ROS_INFO("angle %f", msgs);	

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

	Imu_sub = nh.subscribe("/imu/data_raw", 1 , &Locomotion_Driver::ImuCallback, this);
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "locomotion_startup");
	ros::NodeHandle nh("~");
	
	Locomotion_Driver * node = new Locomotion_Driver(nh);
	ros::spin();
	
	return 0;
} 
