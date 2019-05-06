#include <iostream>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "arm_hw.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller_node");
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    arm_hardware_interface::Arm_interface hw;
    bool started = hw.init(nh,nh);

    controller_manager::ControllerManager cm(&hw,nh);

    ros::Duration period(1.0/5.0);

    ROS_INFO("Started arm control node");
    while(ros::ok()) {
        period.sleep();
    }

    spinner.stop();

    return 0;

}