#pragma once

#include "modbus_ros2_control/grippers/modbus_gripper_base.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include <unordered_map>
#include <string>

namespace modbus_ros2_control {

// 前向声明
class ModbusRtuCommunicator;

/**
 * @brief Jodell RG75 夹爪控制类（USB-485 转接）
 *
 * 通过独立 USB-485 串口适配器驱动 Jodell RG75（与天机 485 走 marvin_ros2_control 并行）。
 * 协议与 marvin_ros2_control 的 JDGripper 一致（slave 0x09，FC04 读状态 / FC10 写命令）：
 * - 命令：3 个保持寄存器 @0x03E8（触发 / 位置 / 力+速度），由 JodellCommandBuilder 生成
 * - 状态：3 个输入寄存器 @0x07D0（reg1 高字节 = 位置，reg2 低字节 = 速度，高字节 = 力）
 */
class JodellGripper : public ModbusGripperBase {
public:
    /**
     * @brief 构造函数
     * @param logger ROS2日志记录器
     * @param clock ROS2时钟
     * @param joint_name 夹爪关节名称
     */
    JodellGripper(
        rclcpp::Logger logger,
        const rclcpp::Clock::SharedPtr& clock,
        const std::string& joint_name = ""
    );

    /**
     * @brief 析构函数
     */
    ~JodellGripper() override;

    /**
     * @brief 初始化夹爪
     * @param communicator Modbus 通信器（已连接）
     * @param params 硬件参数（可选：torque / velocity 归一化默认值 [0,1]）
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
     * @brief 获取 Jodell 夹爪的默认 Modbus 参数
     * @return Modbus 参数结构体
     */
    static ModbusParams getDefaultModbusParams();

private:
    // Modbus 通信器指针
    ModbusRtuCommunicator* communicator_;

    // 归一化默认力/速度（0.0-1.0，1.0 = 最大）
    double torque_ = 1.0;
    double velocity_ = 1.0;
};

} // namespace modbus_ros2_control
