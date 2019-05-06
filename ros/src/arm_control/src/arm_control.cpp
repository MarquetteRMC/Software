#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <ros/console.h>

namespace arm_control
{

    class ArmPositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
            bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
            {
                std::vector<std::string> joints;
                n.getParam("joints", joints)) {
                ROS_DEBUG_STREAM(joints[0])
                h_joint_hw = hw->getHandle(joints[0]);
                p_joint_hw = hw->getHandle(joint[1]);
                return true;
            }

            void update(const ros::Time& time, const ros::Duration& period)
            {
                //double error = setpoint_ - joint_.getPosition();
                //joint_.setCommand(error*gain_);
            }

            void starting(const ros::Time& time) {}
            void stopping(const ros::Time& time) {}

            private:
                hardware_interface::JointHandle h_joint_hw;
                hardware_interface::JointHandle p_joint_hw;
    };
}
PLUGINLIB_DECLARE_CLASS(arm_control, PositionController, arm_controller::PositionController, controller_interface::ControllerBase)