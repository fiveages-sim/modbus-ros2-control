#include "modbus_ros2_control/grippers/modbus_gripper_base.h"
#include <algorithm>
#include <cctype>

namespace modbus_ros2_control {

std::string ModbusGripperBase::detectGripperJoint(
    const std::vector<hardware_interface::ComponentInfo>& joints
) {
    for (const auto& joint : joints) {
        // 检查关节名称中是否包含 gripper 或 hand
        std::string joint_name_lower = joint.name;
        std::transform(
            joint_name_lower.begin(),
            joint_name_lower.end(),
            joint_name_lower.begin(),
            ::tolower
        );

        if (joint_name_lower.find("gripper") != std::string::npos ||
            joint_name_lower.find("hand") != std::string::npos) {
            return joint.name;
        }
    }

    return "";  // 未检测到夹爪
}

ModbusGripperBase::ModbusGripperBase(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    const std::string& joint_name
)
    : logger_(logger)
    , clock_(clock)
    , joint_name_(joint_name)
    , initialized_(false)
    , position_(0.0)
    , velocity_(0.0)
    , effort_(0.0)
    , position_command_(0.0)
    , last_command_(-1.0)
{
}

void ModbusGripperBase::exportStateInterfaces(
    std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces
) {
    state_interfaces.push_back(
        std::make_shared<hardware_interface::StateInterface>(
            joint_name_, hardware_interface::HW_IF_POSITION, getPositionPtr()
        )
    );

    state_interfaces.push_back(
        std::make_shared<hardware_interface::StateInterface>(
            joint_name_, hardware_interface::HW_IF_VELOCITY, getVelocityPtr()
        )
    );

    state_interfaces.push_back(
        std::make_shared<hardware_interface::StateInterface>(
            joint_name_, hardware_interface::HW_IF_EFFORT, getEffortPtr()
        )
    );
}

void ModbusGripperBase::exportCommandInterfaces(
    std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces
) {
    command_interfaces.push_back(
        std::make_shared<hardware_interface::CommandInterface>(
            joint_name_, hardware_interface::HW_IF_POSITION, getPositionCommandPtr()
        )
    );
}

} // namespace modbus_ros2_control

