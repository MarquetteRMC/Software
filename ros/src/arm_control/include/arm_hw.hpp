#include <iostream>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros>

namespace arm_hardware_interface
{
    class Arm_interface: public hardware_interface::RobotHW
    {
    public:
        Arm_interface();
        ~Arm_interface();

    protected:
        ros::NodeHandle nh_;

        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::PositionJointInterface joint_position_interface;
        double cmd[2];
        double pos[2];
        double vel[2];
        double eff[2];
    }
}
