#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "modbus_ros2_control/grippers/modbus_gripper_base.h"
#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace modbus_ros2_control {

/**
 * @brief Modbus 末端执行器 ROS2 Control 硬件接口
 * 
 * 通用的 Modbus 末端执行器硬件接口，支持通过配置选择不同的末端执行器类型
 * 当前支持：
 * - ChangingtekGripper
 * 
 * 未来可以扩展支持其他 Modbus 末端执行器（夹爪、工具等）
 */
class ModbusHardware : public hardware_interface::SystemInterface {
public:
    /**
     * @brief 初始化硬件接口
     * @param params 硬件组件接口参数
     * @return 初始化结果
     */
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params
    ) override;

    /**
     * @brief 激活硬件接口（建立 Modbus 连接、初始化夹爪）
     * @param previous_state 前一个生命周期状态
     * @return 激活结果
     */
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state
    ) override;

    /**
     * @brief 停用硬件接口（断开 Modbus 连接）
     * @param previous_state 前一个生命周期状态
     * @return 停用结果
     */
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state
    ) override;

    /**
     * @brief 导出状态接口（位置、速度、力矩）
     * @return 状态接口列表
     */
    std::vector<hardware_interface::StateInterface::ConstSharedPtr>
    on_export_state_interfaces() override;

    /**
     * @brief 导出命令接口（位置命令）
     * @return 命令接口列表
     */
    std::vector<hardware_interface::CommandInterface::SharedPtr>
    on_export_command_interfaces() override;

    /**
     * @brief 读取夹爪状态
     * @param time 当前时间
     * @param period 距离上次读取的时间间隔
     * @return 读取结果
     */
    hardware_interface::return_type read(
        const rclcpp::Time& time,
        const rclcpp::Duration& period
    ) override;

    /**
     * @brief 写入控制命令到夹爪
     * @param time 当前时间
     * @param period 距离上次写入的时间间隔
     * @return 写入结果
     */
    hardware_interface::return_type write(
        const rclcpp::Time& time,
        const rclcpp::Duration& period
    ) override;

private:
    // Modbus 通信器
    std::unique_ptr<ModbusRtuCommunicator> modbus_communicator_;

    // 夹爪控制器（使用基类指针，支持多态）
    std::unique_ptr<ModbusGripperBase> gripper_;

    // 配置参数
    std::string gripper_type_;      // 夹爪类型（如 "changingtek"）
    std::string serial_port_;       // 串口路径
    uint32_t baudrate_;             // 波特率
    int slave_id_;                  // 从站地址
    char parity_;                   // 校验位
    int data_bits_;                 // 数据位
    int stop_bits_;                 // 停止位

    // 辅助方法
    /**
     * @brief 从硬件参数中读取配置
     * @param params 硬件参数映射
     */
    void loadParameters(const std::unordered_map<std::string, std::string>& params);

    /**
     * @brief 创建夹爪对象（根据 gripper_type_）
     * @return 是否创建成功
     */
    bool createGripper();
};

} // namespace modbus_ros2_control

