#pragma once

#include "modbus_ros2_control/grippers/modbus_gripper_base.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
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
        const rclcpp::Clock::SharedPtr& clock,
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
     * @param variant 夹爪变体 ("90c" 或 "90d"，默认为 "90c")
     * @return Modbus 参数结构体
     */
    static ModbusParams getDefaultModbusParams(const std::string& variant = "90c");

private:
    // Modbus 通信器指针
    ModbusRtuCommunicator* communicator_;
    
    // 夹爪变体类型：90C 或 90D
    enum class Variant {
        Changingtek90C,
        Changingtek90D
    };
    Variant variant_;
    bool profile_motion_registers_sent_{false};
    
    // 根据变体获取反馈寄存器地址
    uint16_t getFeedbackRegAddr() const;
    // 根据变体获取位置寄存器地址
    uint16_t getPosRegAddr() const;
    // 根据变体获取触发寄存器地址
    uint16_t getTriggerRegAddr() const;
    // 根据变体获取从站ID
    uint8_t getSlaveId() const;

    bool writeRegisterChecked(uint16_t addr, uint16_t value, const char* label);
    bool ensureProfileMotionRegisters();
};

} // namespace modbus_ros2_control

