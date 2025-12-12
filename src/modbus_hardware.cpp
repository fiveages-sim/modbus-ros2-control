#include "modbus_ros2_control/modbus_hardware.h"
#include "modbus_ros2_control/grippers/changingtek_gripper.h"
#include "modbus_ros2_control/grippers/modbus_gripper_base.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

namespace modbus_ros2_control
{
    hardware_interface::CallbackReturn ModbusHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params
    )
    {
        if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 读取配置参数
        loadParameters(params.hardware_info.hardware_parameters);

        // 检测夹爪关节
        std::string gripper_joint_name = ModbusGripperBase::detectGripperJoint(
            params.hardware_info.joints
        );

        if (gripper_joint_name.empty())
        {
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
    )
    {
        RCLCPP_INFO(get_node()->get_logger(), "Activating ModbusHardware...");

        // 连接 Modbus
        if (!modbus_communicator_->connect())
        {
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
        ))
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Failed to initialize gripper"
            );
            modbus_communicator_->disconnect();
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "✅ Gripper initialized");

        // 启动后台读取线程
        // 后台线程以20Hz频率持续读取状态和写入命令，避免阻塞控制循环
        gripper_->startBackgroundReading();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ModbusHardware::on_deactivate(
        const rclcpp_lifecycle::State& /* previous_state */
    )
    {
        RCLCPP_INFO(get_node()->get_logger(), "Deactivating ModbusHardware...");

        if (gripper_)
        {
            // 停止后台读取线程
            gripper_->stopBackgroundReading();
            gripper_->shutdown();
        }

        if (modbus_communicator_)
        {
            modbus_communicator_->disconnect();
        }

        RCLCPP_INFO(get_node()->get_logger(), "✅ ModbusHardware deactivated");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr>
    ModbusHardware::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

        if (gripper_ && gripper_->hasGripper())
        {
            gripper_->exportStateInterfaces(state_interfaces);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
    ModbusHardware::on_export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

        if (gripper_ && gripper_->hasGripper())
        {
            gripper_->exportCommandInterfaces(command_interfaces);
        }

        return command_interfaces;
    }

    hardware_interface::return_type ModbusHardware::read(
        const rclcpp::Time& /* time */,
        const rclcpp::Duration& period
    )
    {
        if (!gripper_ || !gripper_->isInitialized())
        {
            return hardware_interface::return_type::ERROR;
        }

        // 根据控制循环的周期设置后台线程的循环间隔（只设置一次）
        // 后台线程频率设为控制循环频率的 1倍（与控制循环相同频率）
        if (gripper_->isBackgroundReadingActive())
        {
            gripper_->updateBackgroundReadingInterval(period);
        }

        // 检查后台读取线程是否在运行
        // 状态数据已经在后台线程中持续更新，这里不需要再次读取
        if (!gripper_->isBackgroundReadingActive())
        {
            RCLCPP_WARN_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                2000,
                "Background reading thread is not active"
            );
            // 即使线程未运行，也返回 OK，因为可能是刚启动或正在关闭
            // 真正的错误会在 on_activate 时检测
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ModbusHardware::write(
        const rclcpp::Time& /* time */,
        const rclcpp::Duration& /* period */
    )
    {
        if (!gripper_ || !gripper_->isInitialized())
        {
            return hardware_interface::return_type::ERROR;
        }

        // 写入操作现在由后台线程处理，这里不需要执行任何操作
        // position_command_ 已经被 ROS2 Control 框架更新
        // 后台线程会检测到变化并执行写入
        // 这样避免了阻塞控制循环

        return hardware_interface::return_type::OK;
    }

    void ModbusHardware::loadParameters(
        const std::unordered_map<std::string, std::string>& params
    )
    {
        // 读取配置参数的辅助函数
        const auto get_param = [&params](const std::string& name, const std::string& default_val)
        {
            auto it = params.find(name);
            return (it != params.end()) ? it->second : default_val;
        };

        // 首先读取 gripper_type
        gripper_type_ = get_param("gripper_type", "changingtek");

        // 从夹爪类获取默认 Modbus 参数
        ModbusParams default_params;
        if (gripper_type_ == "changingtek")
        {
            default_params = ChangingtekGripper::getDefaultModbusParams();
        }
        else
        {
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
        if (it != params.end())
        {
            serial_port_ = it->second;
        }
        it = params.find("baudrate");
        if (it != params.end())
        {
            baudrate_ = std::stoul(it->second);
        }
        it = params.find("slave_id");
        if (it != params.end())
        {
            slave_id_ = std::stoi(it->second);
        }
        it = params.find("parity");
        if (it != params.end())
        {
            std::string parity_str = it->second;
            parity_ = (parity_str.empty()) ? 'N' : parity_str[0];
        }
        it = params.find("data_bits");
        if (it != params.end())
        {
            data_bits_ = std::stoi(it->second);
        }
        it = params.find("stop_bits");
        if (it != params.end())
        {
            stop_bits_ = std::stoi(it->second);
        }
    }

    bool ModbusHardware::createGripper()
    {
        // 目前只支持 Changingtek，未来可以扩展
        if (gripper_type_ == "changingtek")
        {
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
