#include "modbus_ros2_control/modbus_hardware.h"
#include "modbus_ros2_control/grippers/changingtek_gripper.h"
#include "modbus_ros2_control/grippers/modbus_gripper_base.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

namespace modbus_ros2_control {

hardware_interface::CallbackReturn ModbusHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params
) {
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 读取配置参数
    loadParameters(params.hardware_info.hardware_parameters);

    // 检测夹爪关节
    std::string gripper_joint_name = ModbusGripperBase::detectGripperJoint(
        params.hardware_info.joints
    );

    if (gripper_joint_name.empty()) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "No gripper joint found in configuration"
        );
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 创建 Modbus 通信器
    modbus_communicator_ = std::make_unique<ModbusRtuCommunicator>(
        serial_port_,
        baudrate_,
        slave_id_,
        parity_,
        data_bits_,
        stop_bits_
    );

    // 创建夹爪对象
    gripper_ = std::make_unique<ChangingtekGripper>(
        get_node()->get_logger(),
        get_node()->get_clock(),
        gripper_joint_name
    );

    RCLCPP_INFO(
        get_node()->get_logger(),
        "ModbusHardware initialized:"
    );
    RCLCPP_INFO(
        get_node()->get_logger(),
        "  Gripper type: %s",
        gripper_type_.c_str()
    );
    RCLCPP_INFO(
        get_node()->get_logger(),
        "  Serial port: %s",
        serial_port_.c_str()
    );
    RCLCPP_INFO(
        get_node()->get_logger(),
        "  Baudrate: %u",
        baudrate_
    );
    RCLCPP_INFO(
        get_node()->get_logger(),
        "  Slave ID: %d",
        slave_id_
    );
    RCLCPP_INFO(
        get_node()->get_logger(),
        "  Gripper joint: %s",
        gripper_joint_name.c_str()
    );

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusHardware::on_activate(
    const rclcpp_lifecycle::State& /* previous_state */
) {
    RCLCPP_INFO(get_node()->get_logger(), "Activating ModbusHardware...");

    // 连接 Modbus
    if (!modbus_communicator_->connect()) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to connect Modbus: %s",
            modbus_communicator_->getLastError().c_str()
        );
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "✅ Modbus connected");

    // 初始化夹爪
    if (!gripper_->initialize(
            modbus_communicator_.get(),
            info_.hardware_parameters
        )) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to initialize gripper"
        );
        modbus_communicator_->disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "✅ Gripper initialized");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusHardware::on_deactivate(
    const rclcpp_lifecycle::State& /* previous_state */
) {
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating ModbusHardware...");

    if (gripper_) {
        gripper_->shutdown();
    }

    if (modbus_communicator_) {
        modbus_communicator_->disconnect();
    }

    RCLCPP_INFO(get_node()->get_logger(), "✅ ModbusHardware deactivated");

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
ModbusHardware::on_export_state_interfaces() {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

    if (gripper_ && gripper_->hasGripper()) {
        gripper_->exportStateInterfaces(state_interfaces);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
ModbusHardware::on_export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

    if (gripper_ && gripper_->hasGripper()) {
        gripper_->exportCommandInterfaces(command_interfaces);
    }

    return command_interfaces;
}

hardware_interface::return_type ModbusHardware::read(
    const rclcpp::Time& /* time */,
    const rclcpp::Duration& /* period */
) {
    if (!gripper_ || !gripper_->isInitialized()) {
        return hardware_interface::return_type::ERROR;
    }

    if (!gripper_->readStatus()) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            2000,
            "Failed to read gripper status"
        );
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ModbusHardware::write(
    const rclcpp::Time& /* time */,
    const rclcpp::Duration& /* period */
) {
    if (!gripper_ || !gripper_->isInitialized()) {
        return hardware_interface::return_type::ERROR;
    }

    if (!gripper_->writeCommand()) {
        // 命令未变化是正常情况，不报错
        // 只有真正的错误才会返回 ERROR
    }

    return hardware_interface::return_type::OK;
}

void ModbusHardware::loadParameters(
    const std::unordered_map<std::string, std::string>& params
) {
    // 读取配置参数的辅助函数
    const auto get_param = [&params](const std::string& name, const std::string& default_val) {
        auto it = params.find(name);
        return (it != params.end()) ? it->second : default_val;
    };

    // 首先读取 gripper_type
    gripper_type_ = get_param("gripper_type", "changingtek");
    
    // 从夹爪类获取默认 Modbus 参数
    ModbusParams default_params;
    if (gripper_type_ == "changingtek") {
        default_params = ChangingtekGripper::getDefaultModbusParams();
    } else {
        // 未知类型的默认值（通用 Modbus RTU 配置）
        RCLCPP_WARN(
            get_node()->get_logger(),
            "Unknown gripper type '%s', using default Modbus parameters",
            gripper_type_.c_str()
        );
    }
    
    // 使用默认参数初始化
    serial_port_ = default_params.serial_port;
    baudrate_ = default_params.baudrate;
    slave_id_ = default_params.slave_id;
    parity_ = default_params.parity;
    data_bits_ = default_params.data_bits;
    stop_bits_ = default_params.stop_bits;
    
    // 允许用户通过配置文件覆盖默认值
    auto it = params.find("serial_port");
    if (it != params.end()) {
        serial_port_ = it->second;
    }
    it = params.find("baudrate");
    if (it != params.end()) {
        baudrate_ = std::stoul(it->second);
    }
    it = params.find("slave_id");
    if (it != params.end()) {
        slave_id_ = std::stoi(it->second);
    }
    it = params.find("parity");
    if (it != params.end()) {
        std::string parity_str = it->second;
        parity_ = (parity_str.empty()) ? 'N' : parity_str[0];
    }
    it = params.find("data_bits");
    if (it != params.end()) {
        data_bits_ = std::stoi(it->second);
    }
    it = params.find("stop_bits");
    if (it != params.end()) {
        stop_bits_ = std::stoi(it->second);
    }
}

bool ModbusHardware::createGripper() {
    // 目前只支持 Changingtek，未来可以扩展
    if (gripper_type_ == "changingtek") {
        // 已在 on_init 中创建
        return true;
    }

    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Unsupported gripper type: %s",
        gripper_type_.c_str()
    );
    return false;
}

} // namespace modbus_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(
    modbus_ros2_control::ModbusHardware,
    hardware_interface::SystemInterface
)

