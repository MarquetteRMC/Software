#include <arm_hw.hpp>

namespace arm_hardware_interface
{
    Arm_interface::Arm_interface() {

        hardware_interface::JointStateHandle height_handle("base_to_lever_arm", &pos[0], &vel[0], &eff[0]);
        joint_state_interface.registerHandle(height_handle);

        hardware_interface::JointStateHandle pitch_handle("lever_arm_to_digging_arm", &pos[1], &vel[1], &eff[1]);
        joint_state_interface.registerHandle(pitch_handle);

        registerInterface(&joint_state_interface);

        hardware_interface::JointHandle pos_handle_height(joint_state_interface.getHandle("base_to_lever_arm"), &cmd[0]);
        joint_position_interface.registerHandle(pos_handle_height);

        hardware_interface::JointHandle pos_handle_pitch(joint_state_interface.getHandle("base_to_lever_arm"), &cmd[0]);
        joint_position_interface.registerHandle(pos_handle_pitch);

        registerInterface(&joint_position_interface);
    }
}