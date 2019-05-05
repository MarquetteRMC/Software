#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace arm_controller
{

    class ArmPositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
    {
        public:
            bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
            {
                std::string height_joint;
                if (!n.getParam("base_to_lever_arm", height_joint)) {
                    ROS_ERROR("could not find height_joint");
                    return false;
                }

                std::string pitch_joint;
                if (!n.getParam("lever_arm_to_digging_arm", pitch_joint)) {
                    ROS_ERROR("could not find pitch_joint");
                    return false;
                }

                h_joint_hw = hw->getHandle(height_joint);
                p_joint_hw = hw->getHandle(pitch_joint);
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
    PLUGINLIB_DECLARE_CLASS(arm_control, PositionController, arm_controller::PositionController, controller_interface::ControllerBase)
}