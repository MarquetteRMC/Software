/*
This code will subscribe to the /qr_codes topic and determine what command velocities to send to the odrive motors
Author: Nathan Faust
Last Date edit: 4/12/2019
Not sure if really needed anymore. Date:4/26 
*/


#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "opencv/cv.h"
#include <time.h>


bool front_detection = false;
bool back_detection = false;
int front_qr_counter = 1;
int back_qr_counter = 1;
bool Digging = false;
bool Start_Up_Sequence = true;


std::string barcode_data;
std::string M1_encoder;
std::string M2_encoder;


void qr_find_callback(const std_msgs::String qr_msg){
     			
	barcode_data = qr_msg.data;    
}

//need to figure out how to work ViSP before this callback function will be made
/*void IMU_callback(const sensor_msgs::Imu::ConstPtr &ImuData){
    

}*/

//For both this callback and M2 need to figure out how to just recieve the numerical data value, not the extra stuff.
/*void ENCM1_callback(const std_msgs::String M1_encoder){
    
    M1_encoder = M1_encoder.data;

}

void ENCM2_callback(const std_msgs::String M2_encoder){
    
    M2_encoder = M2_encoder.data;

}*/





int main(int argc, char **argv){
    ros::init(argc, argv, "command_velocity_publisher");
    ros::NodeHandle nh;
    ros::Publisher vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    ros::Subscriber qr_sub = nh.subscribe("qr_codes", 10, qr_find_callback);
    //ros::Subscriber IMU_sub = nh.subscribe("imu/data_raw", 10, IMU_callback);
    //ros::Subscriber Pot_ENC1_sub = nh.subscribe("ENCM1_pot_feedback", 3, ENCM1_callback); //future change separate pot feedbacks into one
    //ros::Subsriber Pot_ENC2_sub = nh.subscribe("ENCM2_pot_feedback", 3, ENCM2_callback);
    ros::Rate loop_rate(10);

    geometry_msgs::Twist twist;
  
    while (ros::ok()){
        
        //First step is to spin and find the QR code to initally get the orientation.
        while(Start_Up_Sequence == true){          
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
	            Start_Up_Sequence = false;		            
	            }
	        //if the front qr is detected first spin 270 degrees. same way as other. Not a huge fan
	        if(Back_time > Front_time){
	            my_countdown = my_countdown + (float)6000000.0;
	            while(my_countdown > std::clock()){
	                twist.linear.x =0;
	                twist.angular.z = -100;
                    vel_publisher.publish(twist);
	                }
	            Start_Up_Sequence = false;	
                }          
            ros::spinOnce();
       	    loop_rate.sleep();
            }
           
    }
    }
