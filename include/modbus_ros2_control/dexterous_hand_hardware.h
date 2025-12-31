#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "modbus_ros2_control/hands/dexterous_hand_base.h"
#include "modbus_ros2_control/hands/simple_dexterous_hand_wrapper.h"

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace modbus_ros2_control
{
    /**
     * @brief 灵巧手 ROS2 Control 硬件接口
     * 
     * 支持 SimpleDexterousHand 的 ROS2 Control 硬件接口
     * 提供7个关节的位置、速度、力矩接口
     */
    class DexterousHandHardware : public hardware_interface::SystemInterface
    {
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
         * @brief 激活硬件接口（初始化灵巧手）
         * @param previous_state 前一个生命周期状态
         * @return 激活结果
         */
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state
        ) override;

        /**
         * @brief 停用硬件接口（关闭灵巧手连接）
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
         * @brief 读取灵巧手状态
         * @param time 当前时间
         * @param period 距离上次读取的时间间隔
         * @return 读取结果
         */
        hardware_interface::return_type read(
            const rclcpp::Time& time,
            const rclcpp::Duration& period
        ) override;

        /**
         * @brief 写入控制命令到灵巧手
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

        // 灵巧手控制器（使用基类指针，支持多态）
        std::unique_ptr<DexterousHandBase> hand_;

        // 配置参数
        std::string hand_type_;      // 灵巧手类型（如 "simple_dexterous_hand"）
        std::string hand_side_;      // 手部：left 或 right
        std::string serial_port_;    // 串口路径
        uint8_t modbus_slave_id_;    // Modbus 从站地址（0x27=右手, 0x28=左手）

        // 辅助方法
        /**
         * @brief 从硬件参数中读取配置
         * @param params 硬件参数映射
         */
        void loadParameters(const std::unordered_map<std::string, std::string>& params);

        /**
         * @brief 创建灵巧手对象（根据 hand_type_）
         * @return 是否创建成功
         */
        bool createHand(const std::vector<hardware_interface::ComponentInfo>& joints);
    };
} // namespace modbus_ros2_control

