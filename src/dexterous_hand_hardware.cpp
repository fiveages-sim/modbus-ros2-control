#include "modbus_ros2_control/dexterous_hand_hardware.h"
#include "modbus_ros2_control/hands/simple_dexterous_hand_wrapper.h"
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

        if (hand_joint_names.size() != ModbusConfig::DexterousHand::JOINT_COUNT)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Dexterous hand requires exactly %d joints, found %zu",
                ModbusConfig::DexterousHand::JOINT_COUNT,
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

        // 创建灵巧手对象
        hand_ = std::make_unique<SimpleDexterousHandWrapper>(
            get_node()->get_logger(),
            get_node()->get_clock(),
            hand_joint_names
        );

        RCLCPP_INFO(
            get_node()->get_logger(),
            "DexterousHandHardware initialized:"
        );
        RCLCPP_INFO(
            get_node()->get_logger(),
            "  Hand type: %s",
            hand_type_.c_str()
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

        // 从参数服务器获取 robot_description（launch 文件已传递）
        std::string robot_description;
        
        // 首先尝试声明参数（如果还没有声明）
        if (!get_node()->has_parameter("robot_description"))
        {
            get_node()->declare_parameter<std::string>("robot_description", "");
        }
        
        // 尝试获取参数
        try
        {
            robot_description = get_node()->get_parameter("robot_description").as_string();
            
            if (robot_description.empty())
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "robot_description parameter is empty. Please ensure launch file passes robot_description to ros2_control_node."
                );
                modbus_communicator_->disconnect();
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            RCLCPP_INFO(
                get_node()->get_logger(),
                "✅ Got robot_description from parameter server (%zu bytes)",
                robot_description.size()
            );
        }
        catch (const rclcpp::ParameterTypeException& e)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "robot_description parameter type error: %s. Trying alternative method...",
                e.what()
            );
            
            // 尝试使用 get_parameter 方法
            try
            {
                if (get_node()->get_parameter("robot_description", robot_description))
                {
                    if (robot_description.empty())
                    {
                        RCLCPP_ERROR(
                            get_node()->get_logger(),
                            "robot_description parameter is empty."
                        );
                        modbus_communicator_->disconnect();
                        return hardware_interface::CallbackReturn::ERROR;
                    }
                    RCLCPP_INFO(
                        get_node()->get_logger(),
                        "✅ Got robot_description using get_parameter (%zu bytes)",
                        robot_description.size()
                    );
                }
                else
                {
                    RCLCPP_ERROR(
                        get_node()->get_logger(),
                        "robot_description parameter not found. Please ensure launch file passes robot_description to ros2_control_node."
                    );
                    modbus_communicator_->disconnect();
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }
            catch (const std::exception& e2)
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "Could not get robot_description from parameter server: %s",
                    e2.what()
                );
                modbus_communicator_->disconnect();
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Could not get robot_description from parameter server: %s",
                e.what()
            );
            modbus_communicator_->disconnect();
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 初始化灵巧手
        if (!hand_->initialize(
            modbus_communicator_.get(),
            info_.hardware_parameters,
            robot_description
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

    bool DexterousHandHardware::createHand(const std::vector<hardware_interface::ComponentInfo>& joints)
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

