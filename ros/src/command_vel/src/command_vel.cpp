/*
This code will subscribe to the /qr_codes topic and determine what command velocities to send to the odrive motors
Author: Nathan Faust
Last Date edit: 4/12/2019
*/


#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include "ros/ros.h"
#include "opencv/cv.h"
#include <time.h>


bool front_detection = false;
bool back_detection = false;

int front_qr_counter = 1;
int back_qr_counter = 1;

std::string barcode_data;



void qr_find_callback(const std_msgs::String qr_msg){
     			
	barcode_data = qr_msg.data;    
}

void IMU_callback(const sensor_msgs::Imu::ConstPtr &ImuData){
    

}

int main(int argc, char **argv){
    ros::init(argc, argv, "command_velocity_publisher");
    ros::NodeHandle nh;
    ros::Publisher vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    ros::Subscriber qr_sub = nh.subscribe("qr_codes", 10, qr_find_callback);
    ros::Subscriber IMU_sub = nh.subscribe("imu/data_raw", 10, IMU_callback);
    ros::Rate loop_rate(10);

    geometry_msgs::Twist twist;
  
    while (ros::ok()){
           
        if(back_detection == false || front_detection == false){ 
	       	twist.linear.x = 0;
	        twist.angular.z = -200.0;
	        //twist.angular.z = 0;
	    
	        vel_publisher.publish(twist);
        }
        
            std::clock_t Front_time = 0;
	    std::clock_t Back_time = 0;
	    std::clock_t my_countdown = 0;
	    if(barcode_data == "FRONT" && front_qr_counter == 1){
	        front_qr_counter = 2;
	        front_detection = true;
        	Front_time = std::clock();    
            }
            
        if(barcode_data == "BACK" && back_qr_counter == 1){
                back_qr_counter = 2;
        	Back_time = std::clock();
	        back_detection = true;
	        }
	    my_countdown = std::clock();
       		
		    //if the back qr is detected first spin 90 degrees. Currently uses timer function to do this
	    if(Front_time > Back_time){
	        my_countdown = my_countdown + (float)2000000.0;
	        while(my_countdown > std::clock()){
	            twist.linear.x =0;
	            twist.angular.z = -100;
	            vel_publisher.publish(twist);
	            }
		        
	        }
	    //if the front qr is detected first spin 270 degrees. same way as other. Not a huge fan
	    if(Back_time > Front_time){
	        my_countdown = my_countdown + (float)6000000.0;
	        while(my_countdown > std::clock()){
	            twist.linear.x =0;
	            twist.angular.z = -100;
                    vel_publisher.publish(twist);
	            }
            }
                        
        ros::spinOnce();   
	loop_rate.sleep();
    
        //ros::spin();
        }
    
    }
