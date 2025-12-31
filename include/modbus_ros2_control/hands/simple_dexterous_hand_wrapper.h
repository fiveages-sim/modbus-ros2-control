#pragma once

#include "modbus_ros2_control/hands/dexterous_hand_base.h"
#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include <unordered_map>
#include <memory>
#include <array>

using namespace gripper_hardware_common;

// Forward declaration - SimpleDexterousHand is defined in marvin_ros2_control
// We'll need to include the actual header or create a wrapper
namespace modbus_ros2_control
{
    /**
     * @brief SimpleDexterousHand 的 ROS2 Control 包装器
     *
     * 将 SimpleDexterousHand 适配到 ROS2 Control 框架
     */
    class SimpleDexterousHandWrapper : public DexterousHandBase
    {
    public:
        /**
         * @brief 构造函数
         * @param logger ROS2日志记录器
         * @param clock ROS2时钟
         * @param joint_names 7个关节名称
         */
        SimpleDexterousHandWrapper(
            rclcpp::Logger logger,
            rclcpp::Clock::SharedPtr clock,
            const std::vector<std::string>& joint_names
        );

        /**
         * @brief 析构函数
         */
        ~SimpleDexterousHandWrapper() override;

        /**
         * @brief 初始化灵巧手
         * @param communicator Modbus 通信器（已连接）
         * @param params 硬件参数
         * @param robot_description URDF 字符串（用于解析关节限制）
         * @return 是否初始化成功
         */
        bool initialize(
            ModbusRtuCommunicator* communicator,
            const std::unordered_map<std::string, std::string>& params,
            const std::string& robot_description
        ) override;

        /**
         * @brief 读取灵巧手状态
         * @return 是否成功读取
         */
        bool readStatus() override;

        /**
         * @brief 写入灵巧手命令
         * @return 是否发送了命令
         */
        bool writeCommand() override;

        /**
         * @brief 关闭灵巧手连接
         */
        void shutdown() override;

    private:
        // Modbus 通信器
        ModbusRtuCommunicator* communicator_;
        
        // 手部配置
        uint8_t modbus_slave_id_;  // Modbus ID: 右手0x27, 左手0x28

        // 关节限制（弧度）：从硬件信息中读取
        std::array<double, 7> joint_lower_limits_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double, 7> joint_upper_limits_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // 位置转换：ROS2 Control 弧度值 <-> Modbus 原始值 (0x00-0xFF)
        // 将弧度值映射到 0-255 范围
        uint8_t radiansToRaw(size_t joint_index, double radians) const;
        double rawToRadians(size_t joint_index, uint8_t raw) const;

        /**
         * @brief 将关节名称映射到 Modbus 寄存器索引
         * @param joint_name 关节名称
         * @return Modbus 寄存器索引 (0-6)，如果未找到返回 -1
         * 
         * Modbus 寄存器映射：
         * 0: Thumb_Pitch (thumb_joint1)
         * 1: Thumb_Yaw (thumb_joint2)
         * 2: Index_Pitch (index_joint)
         * 3: Middle_Pitch (middle_joint)
         * 4: Ring_Pitch (ring_joint)
         * 5: Little_Pitch (pinky_joint)
         * 6: Thumb_Roll (thumb_joint3)
         */
        int getModbusRegisterIndex(const std::string& joint_name) const;
    };
} // namespace modbus_ros2_control

