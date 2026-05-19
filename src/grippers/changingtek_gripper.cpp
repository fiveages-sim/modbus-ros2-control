#include "modbus_ros2_control/grippers/changingtek_gripper.h"
#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include "gripper_hardware_common/utils/CommandChangeDetector.h"
#include "gripper_hardware_common/utils/PositionConverter.h"
#include "gripper_hardware_common/utils/TorqueConverter.h"
#include "gripper_hardware_common/utils/VelocityConverter.h"
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
            last_applied_effort_command_ = effort_command_;
            last_applied_velocity_command_ = velocity_command_;
            RCLCPP_INFO(logger_, "Changingtek gripper initialized, initial position: %.3f", position_);
        }
        else
        {
            RCLCPP_WARN(logger_, "Failed to read initial gripper position, using 0.0");
            position_ = 0.0;
            position_command_ = 0.0;
            last_command_ = 0.0;
            last_applied_effort_command_ = effort_command_;
            last_applied_velocity_command_ = velocity_command_;
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

    bool ChangingtekGripper::writeRegisterChecked(const uint16_t addr, const uint16_t value, const char* label)
    {
        if (communicator_->writeRegister(addr, value))
        {
            return true;
        }
        RCLCPP_ERROR_THROTTLE(
            logger_,
            *clock_,
            1000,
            "Failed to write gripper %s: %s",
            label,
            communicator_->getLastError().c_str());
        return false;
    }

    bool ChangingtekGripper::ensureProfileMotionRegisters()
    {
        if (profile_motion_registers_sent_)
        {
            return true;
        }
        if (!writeRegisterChecked(
                Changingtek90C::ACCELERATION_REG, Changingtek90C::DEFAULT_ACCELERATION, "acceleration") ||
            !writeRegisterChecked(
                Changingtek90C::DECELERATION_REG, Changingtek90C::DEFAULT_DECELERATION, "deceleration"))
        {
            return false;
        }
        profile_motion_registers_sent_ = true;
        return true;
    }

    bool ChangingtekGripper::writeCommand()
    {
        if (!initialized_ || !communicator_)
        {
            return false;
        }

        const bool pos_changed = CommandChangeDetector::hasChanged(position_command_, last_command_);
        const bool eff_changed =
            std::isnan(last_applied_effort_command_) ||
            CommandChangeDetector::hasChanged(last_applied_effort_command_, effort_command_, 0.001);
        const bool vel_changed =
            std::isnan(last_applied_velocity_command_) ||
            CommandChangeDetector::hasChanged(last_applied_velocity_command_, velocity_command_, 0.001);
        if (!pos_changed && !eff_changed && !vel_changed)
        {
            return false;
        }

        const double trq_n = std::clamp(effort_command_, 0.0, 1.0);
        const double vel_n = std::clamp(velocity_command_, 0.0, 1.0);
        const int modbus_torque = TorqueConverter::Changingtek90::normalizedToModbus(trq_n);
        const int vel_byte = VelocityConverter::Changingtek90::normalizedToVelocityRegister(vel_n);
        const uint16_t vel_value = static_cast<uint16_t>(vel_byte);
        const uint16_t trq_value = static_cast<uint16_t>(modbus_torque & 0xFFFF);

        const uint16_t target_pos_mm = PositionConverter::Changingtek90::normalizedToModbus(position_command_);
        const uint16_t pos_registers[2] = {0x0000, target_pos_mm};

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

        usleep(100000);

        // 加/减速度为固定配置，生命周期内只写一次
        if (!ensureProfileMotionRegisters())
        {
            return false;
        }

        if (vel_changed &&
            !writeRegisterChecked(Changingtek90C::VELOCITY_REG, vel_value, "velocity"))
        {
            return false;
        }
        if (eff_changed && !writeRegisterChecked(Changingtek90C::TORQUE_REG, trq_value, "torque"))
        {
            return false;
        }

        if (!writeRegisterChecked(getTriggerRegAddr(), Changingtek90C::TRIGGER_VALUE, "trigger"))
        {
            return false;
        }

        last_command_ = position_command_;
        if (vel_changed)
        {
            last_applied_velocity_command_ = velocity_command_;
        }
        if (eff_changed)
        {
            last_applied_effort_command_ = effort_command_;
        }

        RCLCPP_DEBUG(
            logger_,
            "Gripper command sent: pos=%.3f -> %u mm, vel_norm=%.3f->%u, trq_norm=%.3f->%u",
            position_command_,
            target_pos_mm,
            vel_n,
            vel_value,
            trq_n,
            trq_value
        );

        return true;
    }

    void ChangingtekGripper::shutdown()
    {
        if (initialized_)
        {
            // 停止后台读取线程
            stopBackgroundReading();
            profile_motion_registers_sent_ = false;
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
