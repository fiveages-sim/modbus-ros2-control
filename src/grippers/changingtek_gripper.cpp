#include "modbus_ros2_control/grippers/changingtek_gripper.h"
#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include <unistd.h>
#include <cmath>

namespace modbus_ros2_control {

ChangingtekGripper::ChangingtekGripper(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    const std::string& joint_name
)
    : ModbusGripperBase(logger, clock, joint_name)
    , communicator_(nullptr)
    , read_counter_(0)
{
}

ChangingtekGripper::~ChangingtekGripper() {
    shutdown();
}

bool ChangingtekGripper::initialize(
    ModbusRtuCommunicator* communicator,
    const std::unordered_map<std::string, std::string>& /* params */
) {
    if (!communicator || !communicator->isConnected()) {
        RCLCPP_ERROR(logger_, "Modbus communicator not connected");
        return false;
    }

    communicator_ = communicator;

    // 读取初始位置
    if (readStatus()) {
        position_command_ = position_;
        last_command_ = position_;
        RCLCPP_INFO(logger_, "Changingtek gripper initialized, initial position: %.3f", position_);
    } else {
        RCLCPP_WARN(logger_, "Failed to read initial gripper position, using 0.0");
        position_ = 0.0;
        position_command_ = 0.0;
        last_command_ = 0.0;
    }

    initialized_ = true;
    return true;
}

bool ChangingtekGripper::readStatus() {
    if (!initialized_ || !communicator_) {
        return false;
    }

    // 降低读取频率：每 READ_INTERVAL 次循环读取一次
    read_counter_++;
    if (read_counter_ < READ_INTERVAL) {
        return true;  // 跳过本次读取
    }
    read_counter_ = 0;

    // 读取反馈寄存器（2个寄存器，32位）
    uint16_t feedback[2] = {0};
    int rc = communicator_->readHoldingRegisters(FEEDBACK_REG_ADDR, 2, feedback);
    if (rc != 2) {
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
    uint32_t modbus_pos = ((uint32_t)feedback[0] << 16) | feedback[1];

    // 转换为归一化位置（0.0-1.0）
    double new_position = modbusToNormalized(modbus_pos);

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

bool ChangingtekGripper::writeCommand() {
    if (!initialized_ || !communicator_) {
        return false;
    }

    // 检测命令变化（避免频繁发送）
    if (std::abs(position_command_ - last_command_) < COMMAND_THRESHOLD) {
        return false;  // 命令未变化，无需发送
    }

    // 将归一化位置转换为 Modbus 位置值
    uint16_t target_pos_mm = normalizedToModbus(position_command_);

    // 步骤1：设置目标位置（写入2个寄存器）
    uint16_t pos_registers[2] = {
        0x0000,          // 高位寄存器
        target_pos_mm    // 低位寄存器
    };

    int rc = communicator_->writeRegisters(POS_REG_ADDR, 2, pos_registers);
    if (rc != 2) {
        RCLCPP_ERROR_THROTTLE(
            logger_,
            *clock_,
            1000,
            "Failed to write gripper position: %s",
            communicator_->getLastError().c_str()
        );
        return false;
    }

    // 延时确保数据写入
    usleep(500000);  // 500ms

    // 步骤2：触发运动
    if (!communicator_->writeRegister(TRIGGER_REG_ADDR, 0x0001)) {
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

void ChangingtekGripper::shutdown() {
    if (initialized_) {
        initialized_ = false;
        communicator_ = nullptr;
    }
}

uint16_t ChangingtekGripper::normalizedToModbus(double normalized) const {
    // 限制在有效范围内
    normalized = std::max(0.0, std::min(1.0, normalized));

    // Changingtek: 0.0(闭合) -> 9000, 1.0(打开) -> 0
    uint16_t modbus_pos = static_cast<uint16_t>((1.0 - normalized) * MAX_POSITION_MM);
    return modbus_pos;
}

double ChangingtekGripper::modbusToNormalized(uint32_t modbus_pos) const {
    // Changingtek: 0(打开) -> 1.0, 9000(闭合) -> 0.0
    if (modbus_pos > MAX_POSITION_MM) {
        modbus_pos = MAX_POSITION_MM;
    }

    double normalized = 1.0 - (static_cast<double>(modbus_pos) / MAX_POSITION_MM);
    return std::max(0.0, std::min(1.0, normalized));
}

ModbusParams ChangingtekGripper::getDefaultModbusParams() {
    ModbusParams params;
    params.serial_port = "/dev/ttyUSB0";
    params.baudrate = 115200;
    params.slave_id = 1;
    params.parity = 'N';
    params.data_bits = 8;
    params.stop_bits = 1;
    return params;
}

} // namespace modbus_ros2_control

