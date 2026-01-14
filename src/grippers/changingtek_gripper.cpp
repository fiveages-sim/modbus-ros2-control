#include "modbus_ros2_control/grippers/changingtek_gripper.h"
#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include <unistd.h>
#include <cmath>
#include <utility>
#include <algorithm>

#include "gripper_hardware_common/ChangingtekGripper.h"

using namespace gripper_hardware_common;
using namespace ModbusConfig;

namespace modbus_ros2_control
{
    ChangingtekGripper::ChangingtekGripper(
        rclcpp::Logger logger,
        const rclcpp::Clock::SharedPtr& clock,
        const std::string& joint_name
    )
        : ModbusGripperBase(std::move(logger), clock, joint_name)
          , communicator_(nullptr)
          , variant_(Variant::Changingtek90C)  // 默认使用 90C
    {
    }

    ChangingtekGripper::~ChangingtekGripper()
    {
        shutdown();
    }

    bool ChangingtekGripper::initialize(
        ModbusRtuCommunicator* communicator,
        const std::unordered_map<std::string, std::string>& params
    )
    {
        if (!communicator || !communicator->isConnected())
        {
            RCLCPP_ERROR(logger_, "Modbus communicator not connected");
            return false;
        }

        communicator_ = communicator;

        // 根据参数确定使用的变体（90C 或 90D）
        auto it = params.find("variant");
        if (it != params.end())
        {
            std::string variant_str = it->second;
            std::transform(variant_str.begin(), variant_str.end(), variant_str.begin(), ::tolower);
            
            if (variant_str == "90d" || variant_str == "90_d" || variant_str == "changingtek90d")
            {
                variant_ = Variant::Changingtek90D;
                RCLCPP_INFO(logger_, "Using Changingtek 90D configuration");
            }
            else
            {
                variant_ = Variant::Changingtek90C;
                RCLCPP_INFO(logger_, "Using Changingtek 90C configuration (default)");
            }
        }
        else
        {
            variant_ = Variant::Changingtek90C;  // 默认使用 90C
            RCLCPP_INFO(logger_, "Using Changingtek 90C configuration (default, no variant specified)");
        }

        // 读取初始位置
        if (readStatus())
        {
            position_command_ = position_;
            last_command_ = position_;
            RCLCPP_INFO(logger_, "Changingtek gripper initialized, initial position: %.3f", position_);
        }
        else
        {
            RCLCPP_WARN(logger_, "Failed to read initial gripper position, using 0.0");
            position_ = 0.0;
            position_command_ = 0.0;
            last_command_ = 0.0;
        }

        initialized_ = true;
        return true;
    }

    bool ChangingtekGripper::readStatus()
    {
        if (!initialized_ || !communicator_)
        {
            return false;
        }

        // 读取反馈寄存器（2个寄存器，32位）
        uint16_t feedback[2] = {0};
        int rc = communicator_->readHoldingRegisters(getFeedbackRegAddr(), 2, feedback);
        if (rc != 2)
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Failed to read gripper position: %s",
                communicator_->getLastError().c_str()
            );
            return false;
        }

        // 解析位置：执行器位置 = (high << 16) + low
        uint32_t modbus_pos = static_cast<uint32_t>(feedback[0]) << 16 | feedback[1];

        // 使用通用库的位置转换器
        double new_position = PositionConverter::Changingtek90::modbusToNormalized(modbus_pos);

        // 更新状态
        position_ = new_position;

        RCLCPP_DEBUG_THROTTLE(
            logger_,
            *clock_,
            1000,
            "Gripper position updated: %.3f (modbus: %u)",
            position_,
            modbus_pos
        );

        return true;
    }

    bool ChangingtekGripper::writeCommand()
    {
        if (!initialized_ || !communicator_)
        {
            return false;
        }

        // 使用通用库的命令变化检测器
        if (!CommandChangeDetector::hasChanged(position_command_, last_command_))
        {
            return false; // 命令未变化，无需发送
        }

        // 使用通用库的位置转换器
        uint16_t target_pos_mm = PositionConverter::Changingtek90::normalizedToModbus(position_command_);

        // 步骤1：设置目标位置（写入2个寄存器）
        uint16_t pos_registers[2] = {
            0x0000, // 高位寄存器
            target_pos_mm // 低位寄存器
        };

        int rc = communicator_->writeRegisters(getPosRegAddr(), 2, pos_registers);
        if (rc != 2)
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Failed to write gripper position: %s",
                communicator_->getLastError().c_str()
            );
            return false;
        }

        // 延时确保数据写入（减少延时，因为写入在后台线程中）
        usleep(100000); // 100ms（从500ms减少到100ms）

        // 步骤2：触发运动
        if (!communicator_->writeRegister(getTriggerRegAddr(), Changingtek90C::TRIGGER_VALUE))
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Failed to trigger gripper movement: %s",
                communicator_->getLastError().c_str()
            );
            return false;
        }

        last_command_ = position_command_;

        RCLCPP_DEBUG(
            logger_,
            "Gripper command sent: %.3f -> %u mm",
            position_command_,
            target_pos_mm
        );

        return true;
    }

    void ChangingtekGripper::shutdown()
    {
        if (initialized_)
        {
            // 停止后台读取线程
            stopBackgroundReading();
            initialized_ = false;
            communicator_ = nullptr;
        }
    }

    ModbusParams ChangingtekGripper::getDefaultModbusParams(const std::string& variant)
    {
        ModbusParams params;
        
        // 根据变体选择配置
        std::string variant_lower = variant;
        std::transform(variant_lower.begin(), variant_lower.end(), variant_lower.begin(), ::tolower);
        
        if (variant_lower == "90d" || variant_lower == "90_d" || variant_lower == "changingtek90d")
        {
            params.serial_port = Changingtek90D::DEFAULT_SERIAL_PORT;
            params.baudrate = Changingtek90D::DEFAULT_BAUDRATE;
            params.slave_id = Changingtek90D::DEFAULT_SLAVE_ID;
            params.parity = Changingtek90D::DEFAULT_PARITY;
            params.data_bits = Changingtek90D::DEFAULT_DATA_BITS;
            params.stop_bits = Changingtek90D::DEFAULT_STOP_BITS;
        }
        else
        {
            // 默认使用 90C
            params.serial_port = Changingtek90C::DEFAULT_SERIAL_PORT;
            params.baudrate = Changingtek90C::DEFAULT_BAUDRATE;
            params.slave_id = Changingtek90C::DEFAULT_SLAVE_ID;
            params.parity = Changingtek90C::DEFAULT_PARITY;
            params.data_bits = Changingtek90C::DEFAULT_DATA_BITS;
            params.stop_bits = Changingtek90C::DEFAULT_STOP_BITS;
        }
        
        return params;
    }

    uint16_t ChangingtekGripper::getFeedbackRegAddr() const
    {
        return (variant_ == Variant::Changingtek90D) 
            ? Changingtek90D::FEEDBACK_REG_ADDR 
            : Changingtek90C::FEEDBACK_REG_ADDR;
    }

    uint16_t ChangingtekGripper::getPosRegAddr() const
    {
        // 90C 和 90D 使用相同的位置寄存器地址
        return Changingtek90C::POS_REG_ADDR;
    }

    uint16_t ChangingtekGripper::getTriggerRegAddr() const
    {
        // 90C 和 90D 使用相同的触发寄存器地址
        return Changingtek90C::TRIGGER_REG_ADDR;
    }

    uint8_t ChangingtekGripper::getSlaveId() const
    {
        return (variant_ == Variant::Changingtek90D)
            ? Changingtek90D::DEFAULT_SLAVE_ID
            : Changingtek90C::DEFAULT_SLAVE_ID;
    }
} // namespace modbus_ros2_control
