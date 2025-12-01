#pragma once

#include "modbus_ros2_control/grippers/modbus_gripper_base.h"
#include <unordered_map>
#include <string>

namespace modbus_ros2_control {

// 前向声明
class ModbusRtuCommunicator;

/**
 * @brief Changingtek 夹爪控制类
 * 
 * 实现 Changingtek 夹爪的 Modbus RTU 协议
 * 协议特点：
 * - 位置范围：0-9000（0=打开，9000=闭合）
 * - 位置寄存器：0x0102-0x0103（2个寄存器，32位）
 * - 触发寄存器：0x0108
 * - 反馈寄存器：0x060D-0x060E（2个寄存器，32位）
 */
class ChangingtekGripper : public ModbusGripperBase {
public:
    /**
     * @brief 构造函数
     * @param logger ROS2日志记录器
     * @param clock ROS2时钟
     * @param joint_name 夹爪关节名称
     */
    ChangingtekGripper(
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        const std::string& joint_name = ""
    );

    /**
     * @brief 析构函数
     */
    ~ChangingtekGripper() override;

    /**
     * @brief 初始化夹爪
     * @param communicator Modbus 通信器（已连接）
     * @param params 硬件参数
     * @return 是否初始化成功
     */
    bool initialize(
        ModbusRtuCommunicator* communicator,
        const std::unordered_map<std::string, std::string>& params
    ) override;

    /**
     * @brief 读取夹爪状态
     * @return 是否成功读取
     */
    bool readStatus() override;

    /**
     * @brief 写入夹爪命令
     * @return 是否发送了命令
     */
    bool writeCommand() override;

    /**
     * @brief 关闭夹爪连接
     */
    void shutdown() override;

    /**
     * @brief 静态方法：获取 Changingtek 夹爪的默认 Modbus 参数
     * @return Modbus 参数结构体
     */
    static ModbusParams getDefaultModbusParams();

private:
    // Modbus 通信器指针
    ModbusRtuCommunicator* communicator_;

    // Changingtek 协议参数
    static constexpr uint16_t POS_REG_ADDR = 0x0102;        // 位置寄存器地址
    static constexpr uint16_t TRIGGER_REG_ADDR = 0x0108;    // 触发寄存器地址
    static constexpr uint16_t FEEDBACK_REG_ADDR = 0x060D;   // 反馈寄存器地址
    static constexpr uint16_t MAX_POSITION_MM = 9000;       // 最大位置值（mm）

    // 读取计数器（用于降低读取频率）
    int read_counter_;
    static constexpr int READ_INTERVAL = 4;  // 每4次read循环读取一次

    // 命令变化阈值
    static constexpr double COMMAND_THRESHOLD = 0.01;  // 1%的变化阈值

    // 内部辅助方法
    /**
     * @brief 将归一化位置（0.0-1.0）转换为 Modbus 位置值（0-9000）
     * @param normalized 归一化位置（0.0=闭合，1.0=打开）
     * @return Modbus 位置值（0=打开，9000=闭合）
     */
    uint16_t normalizedToModbus(double normalized) const;

    /**
     * @brief 将 Modbus 位置值（0-9000）转换为归一化位置（0.0-1.0）
     * @param modbus_pos Modbus 位置值（0=打开，9000=闭合）
     * @return 归一化位置（0.0=闭合，1.0=打开）
     */
    double modbusToNormalized(uint32_t modbus_pos) const;
};

} // namespace modbus_ros2_control

