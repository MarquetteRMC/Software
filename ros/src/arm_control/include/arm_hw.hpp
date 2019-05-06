#include <iostream>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

namespace arm_hardware_interface
{
    class Arm_interface: public hardware_interface::RobotHW
    {
    public:
        Arm_interface() {}
        ~Arm_interface() {}
        bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override {
            robot_hw_nh.getParam("joints", joint_name);

            num_joints = joint_name.size();

            cmd.resize(num_joints);
            pos.resize(num_joints);
            vel.resize(num_joints);
            eff.resize(num_joints);
            
            for(int i=0; i<num_joints; i++) {
                ROS_DEBUG_STREAM(joint_name[i]);
                hardware_interface::JointStateHandle jointStateHandle(joint_name[i], &pos[i], &vel[i], &eff[i]);
                joint_state_interface.registerHandle(jointStateHandle);

                hardware_interface::JointHandle jointPosHandle(jointStateHandle, &cmd[0]);
                joint_effort_interface.registerHandle(jointPosHandle);
            }

            registerInterface(&joint_state_interface);
            registerInterface(&joint_effort_interface);
        }

    protected:
        ros::NodeHandle nh_;

        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::EffortJointInterface joint_effort_interface;

        int num_joints;
        std::vector<std::string> joint_name;

        std::vector<double> cmd;
        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> eff;


    };
}
PLUGINLIB_EXPORT_CLASS(arm_hardware_interface::Arm_interface, hardware_interface::RobotHW)