#include "modbus_ros2_control/hands/linkerhand/dexterous_hand_hardware.h"
#include "modbus_ros2_control/hands/dexterous_hand_wrapper_template.h"
#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <string>

using namespace gripper_hardware_common;

namespace modbus_ros2_control
{
    hardware_interface::CallbackReturn DexterousHandHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params
    )
    {
        if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 保存硬件参数（用于后续传递给hand_->initialize）
        // 注意：关节限位从ModbusConfig中读取，不再需要robot_description
        hardware_parameters_ = params.hardware_info.hardware_parameters;
        
        // 调试：打印所有硬件参数
        RCLCPP_INFO(get_node()->get_logger(), "Hardware parameters received:");
        for (const auto& [key, value] : hardware_parameters_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "  %s = %s", key.c_str(), value.c_str());
        }
        
        // 读取配置参数
        loadParameters(params.hardware_info.hardware_parameters);

        // 检测灵巧手关节（需要7个关节）
        std::vector<std::string> hand_joint_names = DexterousHandBase::detectHandJoints(
            params.hardware_info.joints
        );

        // 调试信息：打印所有关节名称
        RCLCPP_INFO(
            get_node()->get_logger(),
            "Total joints in hardware info: %zu",
            params.hardware_info.joints.size()
        );
        for (const auto& joint : params.hardware_info.joints)
        {
            RCLCPP_INFO(
                get_node()->get_logger(),
                "  Joint: %s",
                joint.name.c_str()
            );
        }
        RCLCPP_INFO(
            get_node()->get_logger(),
            "Detected hand joints: %zu",
            hand_joint_names.size()
        );
        for (const auto& joint_name : hand_joint_names)
        {
            RCLCPP_INFO(
                get_node()->get_logger(),
                "  Hand joint: %s",
                joint_name.c_str()
            );
        }

        // Check if we have O7 (7 joints) or 6-DOF hand
        if (hand_joint_names.size() != ModbusConfig::DexterousHand::JOINT_COUNT_O7 &&
            hand_joint_names.size() != ModbusConfig::DexterousHand::JOINT_COUNT_O6)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Dexterous hand requires exactly %d (O7) or %d (6-DOF) joints, found %zu",
                ModbusConfig::DexterousHand::JOINT_COUNT_O7,
                ModbusConfig::DexterousHand::JOINT_COUNT_O6,
                hand_joint_names.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 创建 Modbus 通信器
        // 使用 ModbusConfig 中的默认参数（固定值）
        modbus_communicator_ = std::make_unique<ModbusRtuCommunicator>(
            serial_port_,
            ModbusConfig::DexterousHand::DEFAULT_BAUDRATE,      // 固定波特率115200
            modbus_slave_id_,
            ModbusConfig::DexterousHand::DEFAULT_PARITY,        // 无校验 'N'
            ModbusConfig::DexterousHand::DEFAULT_DATA_BITS,    // 数据位8
            ModbusConfig::DexterousHand::DEFAULT_STOP_BITS     // 停止位1
        );

        // Determine product type from hand_type parameter or joint count
        // Default: O7 for 7 joints, O6 for 6 joints (can be overridden by hand_type parameter)
        std::string product_type = hand_type_;
        std::transform(product_type.begin(), product_type.end(), product_type.begin(), ::toupper);
        if (product_type.find("FREEDOM") != std::string::npos)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Freedom hand uses a dedicated hardware plugin: modbus_ros2_control/FreedomRS485Hardware. "
                "Do not use DexterousHandHardware for hand_type=%s.",
                hand_type_.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        // 根据关节数量和产品类型创建对应的灵巧手对象
        if (hand_joint_names.size() == ModbusConfig::DexterousHand::JOINT_COUNT_O7)
        {
            // O7 hand (7 joints)
            hand_ = std::make_unique<SimpleDexterousHandWrapper>(
                get_node()->get_logger(),
                get_node()->get_clock(),
                hand_joint_names
            );
            RCLCPP_INFO(
                get_node()->get_logger(),
                "Creating O7 dexterous hand (7-DOF)"
            );
        }
        else if (hand_joint_names.size() == ModbusConfig::DexterousHand::JOINT_COUNT_O6)
        {
            // 6-DOF hand: determine product based on hand_type parameter
            if (product_type.find("INSPIRE") != std::string::npos || product_type.find("RH56E2") != std::string::npos)
            {
                hand_ = std::make_unique<InspireE2DexterousHandWrapper>(
                    get_node()->get_logger(),
                    get_node()->get_clock(),
                    hand_joint_names
                );
                RCLCPP_INFO(
                    get_node()->get_logger(),
                    "Creating InspireE2 dexterous hand (6-DOF)"
                );
            }
            else if (product_type == "L6" || product_type.find("L6") != std::string::npos)
            {
                // L6 hand (6 joints)
                hand_ = std::make_unique<L6DexterousHandWrapper>(
                    get_node()->get_logger(),
                    get_node()->get_clock(),
                    hand_joint_names
                );
                RCLCPP_INFO(
                    get_node()->get_logger(),
                    "Creating L6 dexterous hand (6-DOF)"
                );
            }
            else
            {
                // O6 hand (6 joints) - default for 6-DOF
                hand_ = std::make_unique<O6DexterousHandWrapper>(
                    get_node()->get_logger(),
                    get_node()->get_clock(),
                    hand_joint_names
                );
                RCLCPP_INFO(
                    get_node()->get_logger(),
                    "Creating O6 dexterous hand (6-DOF)"
                );
            }
        }

        RCLCPP_INFO(
            get_node()->get_logger(),
            "DexterousHandHardware initialized:"
        );
        // Determine product name for logging
        std::string product_name = "Unknown";
        if (hand_joint_names.size() == ModbusConfig::DexterousHand::JOINT_COUNT_O7)
        {
            product_name = "O7";
        }
        else if (hand_joint_names.size() == ModbusConfig::DexterousHand::JOINT_COUNT_O6)
        {
            std::string product_type = hand_type_;
            std::transform(product_type.begin(), product_type.end(), product_type.begin(), ::toupper);
            if (product_type.find("INSPIRE") != std::string::npos || product_type.find("RH56E2") != std::string::npos)
            {
                product_name = "InspireE2";
            }
            else
            {
                product_name = (product_type == "L6" || product_type.find("L6") != std::string::npos) ? "L6" : "O6";
            }
        }
        
        RCLCPP_INFO(
            get_node()->get_logger(),
            "  Hand type: %s (Product: %s, %zu-DOF)",
            hand_type_.c_str(),
            product_name.c_str(),
            hand_joint_names.size()
        );
        RCLCPP_INFO(
            get_node()->get_logger(),
            "  Serial port: %s",
            serial_port_.c_str()
        );
        RCLCPP_INFO(
            get_node()->get_logger(),
            "  Modbus ID: 0x%02X (%s)",
            modbus_slave_id_,
            (modbus_slave_id_ == 0x27) ? "RIGHT_HAND" : "LEFT_HAND"
        );
        RCLCPP_INFO(
            get_node()->get_logger(),
            "  Joints: %zu",
            hand_joint_names.size()
        );

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DexterousHandHardware::on_activate(
        const rclcpp_lifecycle::State& /* previous_state */
    )
    {
        RCLCPP_INFO(get_node()->get_logger(), "Activating DexterousHandHardware...");

        // 连接 Modbus
        RCLCPP_INFO(
            get_node()->get_logger(),
            "Attempting to connect Modbus: port=%s, slave_id=0x%02X",
            serial_port_.c_str(),
            modbus_slave_id_
        );
        
        if (!modbus_communicator_->connect())
        {
            std::string error_msg = modbus_communicator_->getLastError();
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Failed to connect Modbus: %s",
                error_msg.c_str()
            );
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Please check: 1) Serial port exists and has correct permissions, 2) Device is powered on, 3) Serial port is not used by another process"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "✅ Modbus connected");

        // 初始化灵巧手
        // 使用保存的硬件参数（包含max_speed_ratio等所有参数）
        // 关节限位从ModbusConfig中读取（参考Jodell RG75的实现方式）
        if (!hand_->initialize(
            modbus_communicator_.get(),
            hardware_parameters_
        ))
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Failed to initialize dexterous hand"
            );
            modbus_communicator_->disconnect();
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "✅ Dexterous hand initialized");

        // 再次读取位置并同步命令接口（确保命令接口值与实际位置一致）
        // 这很重要，因为控制器可能在on_activate时设置了默认命令值
        RCLCPP_INFO(get_node()->get_logger(), "Synchronizing command interfaces with actual positions...");
        if (hand_->readStatus())
        {
            // 同步命令接口：将命令位置设置为当前实际位置
            for (size_t i = 0; i < hand_->getJointNames().size(); ++i)
            {
                double* cmd_ptr = hand_->getPositionCommandPtr(i);
                double* pos_ptr = hand_->getPositionPtr(i);
                if (cmd_ptr && pos_ptr)
                {
                    *cmd_ptr = *pos_ptr;
                    RCLCPP_DEBUG(
                        get_node()->get_logger(),
                        "Synchronized joint %s: command = %.4f (from position %.4f)",
                        hand_->getJointNames()[i].c_str(),
                        *cmd_ptr,
                        *pos_ptr
                    );
                }
            }
            RCLCPP_INFO(get_node()->get_logger(), "✅ Command interfaces synchronized with actual positions");
        }
        else
        {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to read positions for synchronization");
        }

        // 启动后台读取线程
        hand_->startBackgroundReading();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DexterousHandHardware::on_deactivate(
        const rclcpp_lifecycle::State& /* previous_state */
    )
    {
        RCLCPP_INFO(get_node()->get_logger(), "Deactivating DexterousHandHardware...");

        if (hand_)
        {
            // 停止后台读取线程
            hand_->stopBackgroundReading();
            hand_->shutdown();
        }

        if (modbus_communicator_)
        {
            modbus_communicator_->disconnect();
        }

        RCLCPP_INFO(get_node()->get_logger(), "✅ DexterousHandHardware deactivated");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr>
    DexterousHandHardware::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

        if (hand_ && hand_->hasHand())
        {
            hand_->exportStateInterfaces(state_interfaces);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
    DexterousHandHardware::on_export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

        if (hand_ && hand_->hasHand())
        {
            hand_->exportCommandInterfaces(command_interfaces);
        }

        return command_interfaces;
    }

    hardware_interface::return_type DexterousHandHardware::read(
        const rclcpp::Time& /* time */,
        const rclcpp::Duration& period
    )
    {
        if (!hand_ || !hand_->isInitialized())
        {
            return hardware_interface::return_type::ERROR;
        }

        // 根据控制循环的周期设置后台线程的循环间隔（只设置一次）
        if (hand_->isBackgroundReadingActive())
        {
            hand_->updateBackgroundReadingInterval(period);
        }

        // 检查后台读取线程是否在运行
        if (!hand_->isBackgroundReadingActive())
        {
            RCLCPP_WARN_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                2000,
                "Background reading thread is not active"
            );
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DexterousHandHardware::write(
        const rclcpp::Time& /* time */,
        const rclcpp::Duration& /* period */
    )
    {
        if (!hand_ || !hand_->isInitialized())
        {
            return hardware_interface::return_type::ERROR;
        }

        // 写入操作由后台线程处理，这里不需要执行任何操作
        return hardware_interface::return_type::OK;
    }

    void DexterousHandHardware::loadParameters(
        const std::unordered_map<std::string, std::string>& params
    )
    {
        // 读取配置参数的辅助函数
        const auto get_param = [&params](const std::string& name, const std::string& default_val)
        {
            auto it = params.find(name);
            return (it != params.end()) ? it->second : default_val;
        };

        // 读取手部类型
        hand_type_ = get_param("hand_type", "simple_dexterous_hand");

        // 读取手部配置（left/right）
        hand_side_ = get_param("hand_side", "right");
        
        // 根据手部配置设置 Modbus ID
        std::string side_lower = hand_side_;
        std::transform(side_lower.begin(), side_lower.end(), side_lower.begin(), ::tolower);
        
        if (side_lower == "left" || side_lower == "left_hand")
        {
            modbus_slave_id_ = ModbusConfig::DexterousHand::LEFT_HAND_SLAVE_ID;  // 左手
        }
        else
        {
            modbus_slave_id_ = ModbusConfig::DexterousHand::RIGHT_HAND_SLAVE_ID;  // 右手（默认）
        }

        // 读取串口配置
        serial_port_ = get_param("serial_port", "/dev/ttyUSB0");
        
        // 波特率固定为115200，不支持修改
        // 其他串口参数也固定：停止位1, 数据位8, 校验位N
    }

    bool DexterousHandHardware::createHand(const std::vector<hardware_interface::ComponentInfo>& /* joints */)
    {
        // 已在 on_init 中创建
        return true;
    }
} // namespace modbus_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(
    modbus_ros2_control::DexterousHandHardware,
    hardware_interface::SystemInterface
)
