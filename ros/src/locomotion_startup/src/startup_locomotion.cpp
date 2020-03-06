#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <math.h>

#define PI 3.14159265


using namespace std;
double angle_to_drive;
double angle;
bool aruco_detected = false;
double angular_velocity;
double time_change;
int Id_Number;

class Locomotion_Driver{
	private: 
		ros::Publisher * command_vel_pub;
		ros::Subscriber aruco_sub;
		ros::Subscriber Imu_sub;

		void fiducialsCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr & msg);
		void ImuCallback(const sensor_msgs::Imu::ConstPtr & msg);
		
	public:
		Locomotion_Driver(ros::NodeHandle &nh);
//		ros::Publisher * command_vel_pub;
};


//building this to keep running until aruco method can no longer be seen. 
//Finds the angle between the center of the aruco marker and the rgb camera
void Locomotion_Driver::fiducialsCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
	try {
		double last_x;
                double last_y;
		for (size_t i=0; i<msg->transforms.size(); i++){
			const fiducial_msgs::FiducialTransform &ft = msg->transforms[i];
	
			tf2::Vector3 Vec3(ft.transform.translation.x, ft.transform.translation.y, ft.transform.translation.z);
		
			tf2::Quaternion q(ft.transform.rotation.x,
    				 ft.transform.rotation.y,
	                	 ft.transform.rotation.z,
        	        	 ft.transform.rotation.w);
			
//			ROS_INFO("rot x %f rot y %f rot z %f" , q.x(), q.y(), q.z());
//			ROS_INFO("vec x %f vec y %f vec z %f" , Vec3.x(), Vec3.y(), Vec3.z());
			last_x = Vec3.x();
			last_y = Vec3.y();
			Id_Number = ft.fiducial_id;
			ROS_INFO("ID NUMBER %d", Id_Number);
			
		}
		//convert the x and y vectors into an angle. Subtract 45 since is straight ahead
		angle = (atan(last_x / last_y) * 180/PI);
		angle = angle - 45.0;
		angle_to_drive = 90.0 - angle;

		ROS_INFO("The angle of the marker is %f degrees." , angle);

	}
	catch(...){
	   ROS_INFO("Default Exception\n");
	}
	
}

void Locomotion_Driver::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
	try{
		//assuming the anglular velocity to turn is on the yaw axis so z. determine in testing
		//publishing at a rate of 250 Hz 
		//angular_velocity = change of degree / change of time
		angular_velocity = msg->angular_velocity.z;
		time_change = 1/250; //need to change this to the imu_pub hz value as it fluctuates around 250
	}
	
	catch(...){
	    ROS_INFO("Default Exception\n");
	}

}


Locomotion_Driver::Locomotion_Driver(ros::NodeHandle & nh) {

	aruco_sub = nh.subscribe("/fiducial_transforms", 1, 
			&Locomotion_Driver::fiducialsCallback, this);
		
	Imu_sub = nh.subscribe("/imu/data_raw", 1 , &Locomotion_Driver::ImuCallback, this);
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "locomotion_startup");
	ros::NodeHandle nh("~");
	
	Locomotion_Driver * node = new Locomotion_Driver(nh);

        ros::Publisher command_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	//for first aruco marker. Prevents timeout_counter from stopping if marker isn't found right away
	bool aruco_driving_method = true;
	bool Imu_driving_method = false;
        int timeout_counter;
        int angular_speed;
        geometry_msgs::Twist twist;
	ros::Rate rate(2);
	while(ros::ok()){
	if ( aruco_driving_method == true) {
		ROS_INFO("my id number is %d" , Id_Number);
                //spin right away
                if(aruco_detected == false){
                        ROS_INFO("printing inital spin");
                        angular_speed = 22.0;
                        twist.angular.z = angular_speed;
                        command_vel_pub.publish(twist);
                }
		if(Id_Number != 0.0){
                        ROS_INFO("aruco it is detecting anything");
                        aruco_detected = true;
                }

                //check if aruco gets missed. Prevent it from jerking
                if(Id_Number == 0 && aruco_detected == true){
                        timeout_counter = timeout_counter + 1;
                }
                //keep spinning 
                if(Id_Number != 0 && timeout_counter <= 5){
                        angular_speed = 22.0;
                        twist.angular.z = angular_speed;
                        command_vel_pub.publish(twist);
                        timeout_counter = 0;
                }
                if (aruco_detected == true && timeout_counter > 5){
			ROS_INFO("looping through angular 0");
                        angular_speed = 0;
                        twist.angular.z = angular_speed;
                        command_vel_pub.publish(twist);
			aruco_driving_method = false;
			Imu_driving_method = true;
                }
	}
	if ( Imu_driving_method == true ) {
		 if (angle_to_drive > 0){
                        double theta;
                        theta = angular_velocity * time_change;
                        angle_to_drive = angle_to_drive - theta;
                        angular_speed = 22;
                        twist.angular.z = angular_speed;
                        command_vel_pub.publish(twist);
	                ROS_INFO("angle to drive %f", angle_to_drive);
                }
                //drive forward
		//For future years, this needs to change. currently drives forward till we manually kill it. Needs to figure out how far it has driven and eventually stop in the digging area
                else if(angle_to_drive < 0){
                        int drive_forward;
                        drive_forward = 30;
                        twist.linear.x = drive_forward;
                        command_vel_pub.publish(twist);
                }

	}
	rate.sleep();	
	ros::spinOnce();
	}
	//ros::spin();
	
	return 0;
} 
