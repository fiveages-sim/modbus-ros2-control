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

namespace modbus_ros2_control
{
    // 前向声明
    class ModbusRtuCommunicator;

    /**
     * @brief Modbus 通信参数结构体
     */
    struct ModbusParams
    {
        std::string serial_port = "/dev/ttyUSB0";
        uint32_t baudrate = 115200;
        int slave_id = 1;
        char parity = 'N';
        int data_bits = 8;
        int stop_bits = 1;
    };

    /**
     * @brief Modbus 夹爪抽象基类
     *
     * 定义所有 Modbus 夹爪的统一接口，不同协议的夹爪可以继承此类并实现具体协议
     */
    class ModbusGripperBase
    {
    public:
        /**
         * @brief 静态方法：检测关节配置中是否包含夹爪
         * @param joints 关节信息列表
         * @return 夹爪关节名称，如果未检测到则返回空字符串
         */
        static std::string detectGripperJoint(const std::vector<hardware_interface::ComponentInfo>& joints);


        /**
         * @brief 构造函数
         * @param logger ROS2日志记录器
         * @param clock ROS2时钟（用于限流日志）
         * @param joint_name 夹爪关节名称（如果为空字符串，则表示未配置夹爪）
         */
        ModbusGripperBase(
            rclcpp::Logger logger,
            rclcpp::Clock::SharedPtr clock,
            const std::string& joint_name = ""
        );

        /**
         * @brief 虚析构函数
         * 确保后台线程被正确停止
         */
        virtual ~ModbusGripperBase();

        /**
         * @brief 初始化夹爪（纯虚函数，由子类实现）
         * @param communicator Modbus 通信器（已连接）
         * @param params 硬件参数（从 URDF 配置中读取）
         * @return 是否初始化成功
         */
        virtual bool initialize(
            ModbusRtuCommunicator* communicator,
            const std::unordered_map<std::string, std::string>& params
        ) = 0;

        /**
         * @brief 读取夹爪状态（纯虚函数，由子类实现）
         * @return 是否成功读取
         */
        virtual bool readStatus() = 0;

        /**
         * @brief 写入夹爪命令（纯虚函数，由子类实现）
         * @return 是否发送了命令
         */
        virtual bool writeCommand() = 0;

        /**
         * @brief 关闭夹爪连接（纯虚函数，由子类实现）
         */
        virtual void shutdown() = 0;

        /**
         * @brief 启动后台读取线程
         * 后台线程以固定频率（20Hz）持续读取状态和写入命令，避免阻塞控制循环
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
        bool hasGripper() const { return !joint_name_.empty(); }
        const std::string& getJointName() const { return joint_name_; }
        bool isInitialized() const { return initialized_; }

        // 状态数据指针（用于ROS2 Control接口）
        // 位置：0.0（闭合）到 1.0（打开）
        double* getPositionPtr() { return &position_; }
        double* getVelocityPtr() { return &velocity_; }
        double* getEffortPtr() { return &effort_; }
        double* getPositionCommandPtr() { return &position_command_; }

        /**
         * @brief 导出夹爪状态接口到硬件接口列表
         * @param state_interfaces 状态接口列表（会被修改，添加夹爪接口）
         */
        void exportStateInterfaces(
            std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces
        );

        /**
         * @brief 导出夹爪命令接口到硬件接口列表
         * @param command_interfaces 命令接口列表（会被修改，添加夹爪接口）
         */
        void exportCommandInterfaces(
            std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces
        );

    protected:
        // 日志相关
        rclcpp::Logger logger_;
        rclcpp::Clock::SharedPtr clock_;

        // 夹爪配置
        std::string joint_name_;
        bool initialized_;

        // 夹爪状态（归一化值：0.0=闭合，1.0=打开）
        // 注意：后台线程写入，ROS2 Control框架读取
        // double类型在64位系统上读写通常是原子的，无需锁保护
        double position_ = 0.0; // 当前位置
        double velocity_ = 0.0; // 当前速度
        double effort_ = 0.0; // 当前力矩
        double position_command_ = 0.0; // 位置命令
        double last_command_ = -1.0; // 上一次命令（用于检测变化）

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
