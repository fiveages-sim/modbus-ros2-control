#pragma once

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <chrono>
#include <array>
#include <mutex>

// Forward declaration
namespace modbus_ros2_control {
    class ModbusRtuCommunicator;
}

namespace modbus_ros2_control
{
    /**
     * @brief 灵巧手抽象基类
     *
     * 定义所有灵巧手的统一接口，适配 ROS2 Control 框架
     */
    class DexterousHandBase
    {
    public:
        /**
         * @brief 静态方法：检测关节配置中是否包含灵巧手关节
         * @param joints 关节信息列表
         * @return 灵巧手关节名称列表，如果未检测到则返回空列表
         */
        static std::vector<std::string> detectHandJoints(
            const std::vector<hardware_interface::ComponentInfo>& joints
        );

        /**
         * @brief 构造函数
         * @param logger ROS2日志记录器
         * @param clock ROS2时钟（用于限流日志）
         * @param joint_names 灵巧手关节名称列表（7个关节）
         */
        DexterousHandBase(
            rclcpp::Logger logger,
            rclcpp::Clock::SharedPtr clock,
            const std::vector<std::string>& joint_names
        );

        /**
         * @brief 虚析构函数
         * 确保后台线程被正确停止
         */
        virtual ~DexterousHandBase();

        /**
         * @brief 初始化灵巧手（纯虚函数，由子类实现）
         * @param communicator Modbus 通信器（已连接）
         * @param params 硬件参数（从 URDF 配置中读取）
         * @param robot_description URDF 字符串（用于解析关节限制）
         * @return 是否初始化成功
         */
        virtual bool initialize(
            ModbusRtuCommunicator* communicator,
            const std::unordered_map<std::string, std::string>& params,
            const std::string& robot_description
        ) = 0;

        /**
         * @brief 读取灵巧手状态（纯虚函数，由子类实现）
         * @return 是否成功读取
         */
        virtual bool readStatus() = 0;

        /**
         * @brief 写入灵巧手命令（纯虚函数，由子类实现）
         * @return 是否发送了命令
         */
        virtual bool writeCommand() = 0;

        /**
         * @brief 关闭灵巧手连接（纯虚函数，由子类实现）
         */
        virtual void shutdown() = 0;

        /**
         * @brief 启动后台读取线程
         * 后台线程以固定频率持续读取状态和写入命令，避免阻塞控制循环
         */
        void startBackgroundReading();

        /**
         * @brief 停止后台读取线程
         */
        void stopBackgroundReading();

        /**
         * @brief 检查后台读取线程是否正在运行
         */
        bool isBackgroundReadingActive() const { return reading_thread_active_; }

        /**
         * @brief 更新后台线程的循环间隔（根据控制循环频率动态调整）
         * @param period 控制循环的周期（Duration）
         */
        void updateBackgroundReadingInterval(const rclcpp::Duration& period);

        // 访问器方法
        bool hasHand() const { return !joint_names_.empty(); }
        const std::vector<std::string>& getJointNames() const { return joint_names_; }
        bool isInitialized() const { return initialized_; }

        // 状态数据指针（用于ROS2 Control接口）
        // 位置：0.0 到 1.0（归一化值）
        // 7个关节的位置、速度、力矩
        double* getPositionPtr(size_t joint_index) 
        { 
            if (joint_index < 7) return &positions_[joint_index];
            return nullptr;
        }
        double* getVelocityPtr(size_t joint_index) 
        { 
            if (joint_index < 7) return &velocities_[joint_index];
            return nullptr;
        }
        double* getEffortPtr(size_t joint_index) 
        { 
            if (joint_index < 7) return &efforts_[joint_index];
            return nullptr;
        }
        double* getPositionCommandPtr(size_t joint_index) 
        { 
            if (joint_index < 7) return &position_commands_[joint_index];
            return nullptr;
        }

        /**
         * @brief 导出灵巧手状态接口到硬件接口列表
         * @param state_interfaces 状态接口列表（会被修改，添加灵巧手接口）
         */
        void exportStateInterfaces(
            std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces
        );

        /**
         * @brief 导出灵巧手命令接口到硬件接口列表
         * @param command_interfaces 命令接口列表（会被修改，添加灵巧手接口）
         */
        void exportCommandInterfaces(
            std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces
        );

    protected:
        // 日志相关
        rclcpp::Logger logger_;
        rclcpp::Clock::SharedPtr clock_;

        // 灵巧手配置
        std::vector<std::string> joint_names_;  // 7个关节名称
        bool initialized_;

        // 灵巧手状态（归一化值：0.0 到 1.0）
        // 注意：后台线程写入，ROS2 Control框架读取
        std::array<double, 7> positions_ = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};  // 当前位置
        std::array<double, 7> velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 当前速度
        std::array<double, 7> efforts_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      // 当前力矩
        std::array<double, 7> position_commands_ = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};  // 位置命令
        std::array<double, 7> last_commands_ = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};  // 上一次命令

        // 后台读取线程管理
        std::thread reading_thread_;
        std::atomic<bool> reading_thread_active_{false};
        std::atomic<bool> reading_thread_stop_{false};
        std::atomic<int> loop_interval_ms_{50}; // 动态循环间隔（默认50ms，对应20Hz）
        std::atomic<bool> interval_initialized_{false}; // 是否已初始化间隔

        /**
         * @brief 后台读取和写入线程函数
         * 该线程同时处理读取状态和写入命令，避免阻塞控制循环
         */
        void backgroundReadingLoop();
    };
} // namespace modbus_ros2_control

