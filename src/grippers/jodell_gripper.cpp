#include "modbus_ros2_control/grippers/jodell_gripper.h"
#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include "gripper_hardware_common/JodellGripper.h"
#include <algorithm>
#include <utility>
#include <thread>
#include <chrono>

using namespace gripper_hardware_common;
using namespace gripper_hardware_common::ModbusConfig;

namespace modbus_ros2_control
{
    JodellGripper::JodellGripper(
        rclcpp::Logger logger,
        const rclcpp::Clock::SharedPtr& clock,
        const std::string& joint_name
    )
        : ModbusGripperBase(std::move(logger), clock, joint_name)
          , communicator_(nullptr)
    {
    }

    JodellGripper::~JodellGripper()
    {
        shutdown();
    }

    bool JodellGripper::initialize(
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

        // 可选：归一化默认力 / 速度（0.0-1.0）
        auto it = params.find("torque");
        if (it != params.end())
        {
            torque_ = std::clamp(std::stod(it->second), 0.0, 1.0);
        }
        it = params.find("velocity");
        if (it != params.end())
        {
            velocity_ = std::clamp(std::stod(it->second), 0.0, 1.0);
        }

        // 初始化写：与天机485（marvin JDGripper）一致 fire-and-forget，
        // Jodell 对 FC10 写可能不回 ACK，因此不等待响应、不以写超时作为失败依据。
        const uint16_t init_value = Jodell::INIT_VALUE;
        if (!communicator_->writeRegistersNoAck(Jodell::INIT_REG_ADDR, 1, &init_value))
        {
            RCLCPP_WARN(
                logger_,
                "Jodell init write could not be sent: %s",
                communicator_->getLastError().c_str()
            );
        }
        else
        {
            RCLCPP_INFO(logger_, "Jodell init write sent (slave: 0x%02X)", Jodell::SLAVE_ADDRESS);
        }

        // 用状态读（FC04 @0x07D0）确认链路真正可用 —— 这才是真正的通信测试
        initialized_ = true;
        bool link_ok = false;
        for (int attempt = 0; attempt < 3 && !link_ok; ++attempt)
        {
            link_ok = readStatus();
            if (!link_ok && attempt < 2)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
        if (!link_ok)
        {
            initialized_ = false;
            RCLCPP_ERROR(
                logger_,
                "Jodell gripper not responding on %s (baudrate=%u, slave=0x%02X): status read timed out. "
                "Check wiring / baudrate / slave_id / serial_port.",
                communicator_->getSerialPort().c_str(),
                communicator_->getBaudrate(),
                communicator_->getSlaveId()
            );
            return false;
        }

        position_command_ = position_;
        last_command_ = position_;
        RCLCPP_INFO(logger_, "Jodell gripper link OK, initial position: %.3f", position_);
        return true;
    }

    bool JodellGripper::readStatus()
    {
        if (!initialized_ || !communicator_)
        {
            return false;
        }

        uint16_t registers[Jodell::STATUS_REG_COUNT] = {0};
        const int rc = communicator_->readInputRegisters(
            Jodell::STATUS_REG_ADDR, Jodell::STATUS_REG_COUNT, registers
        );
        if (rc != Jodell::STATUS_REG_COUNT)
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Failed to read Jodell gripper status: %s",
                communicator_->getLastError().c_str()
            );
            return false;
        }

        // reg1 高字节 = 位置（0-255）；reg2 低字节 = 速度，高字节 = 力
        const int status_reg_high = static_cast<int>(registers[1] >> 8);
        position_ = PositionConverter::Jodell::jodellToNormalized(status_reg_high);
        velocity_ = static_cast<int>(registers[2] & 0xFF) / 255.0;
        effort_ = static_cast<int>(registers[2] >> 8) / 255.0;

        return true;
    }

    bool JodellGripper::writeCommand()
    {
        if (!initialized_ || !communicator_)
        {
            return false;
        }

        if (!CommandChangeDetector::hasChanged(position_command_, last_command_))
        {
            return false; // 命令未变化，无需发送
        }

        // 归一化命令 → Jodell 线值（与 marvin JDGripper::move_gripper 一致）
        const int pos_set = PositionConverter::Jodell::normalizedToJodell(position_command_);
        const int jodell_torque = TorqueConverter::Jodell::normalizedToJodell(torque_);
        const int vel_set = VelocityConverter::Jodell::normalizedToLowByte(velocity_);
        const auto position_values = JodellCommandBuilder::buildCommand(pos_set, jodell_torque, vel_set);

        // fire-and-forget（与天机485一致）：不等待写 ACK
        if (!communicator_->writeRegistersNoAck(
                Jodell::POSITION_REG_ADDR,
                static_cast<int>(position_values.size()),
                position_values.data()))
        {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Failed to send Jodell gripper command: %s",
                communicator_->getLastError().c_str()
            );
            return false;
        }

        last_command_ = position_command_;
        return true;
    }

    void JodellGripper::shutdown()
    {
        if (initialized_)
        {
            // 停止后台读取线程
            stopBackgroundReading();
            initialized_ = false;
            communicator_ = nullptr;
        }
    }

    ModbusParams JodellGripper::getDefaultModbusParams()
    {
        ModbusParams params;
        params.serial_port = "/dev/ttyUSB0";
        params.baudrate = 115200;
        params.slave_id = Jodell::SLAVE_ADDRESS;
        params.parity = 'N';
        params.data_bits = 8;
        params.stop_bits = 1;
        return params;
    }
} // namespace modbus_ros2_control
