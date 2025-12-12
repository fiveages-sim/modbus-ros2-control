#include "modbus_ros2_control/grippers/modbus_gripper_base.h"
#include <algorithm>
#include <cctype>
#include <utility>
#include <chrono>
#include <thread>

namespace modbus_ros2_control
{
    std::string ModbusGripperBase::detectGripperJoint(
        const std::vector<hardware_interface::ComponentInfo>& joints
    )
    {
        for (const auto& joint : joints)
        {
            // 检查关节名称中是否包含 gripper 或 hand
            std::string joint_name_lower = joint.name;
            std::transform(
                joint_name_lower.begin(),
                joint_name_lower.end(),
                joint_name_lower.begin(),
                tolower
            );

            if (joint_name_lower.find("gripper") != std::string::npos ||
                joint_name_lower.find("hand") != std::string::npos)
            {
                return joint.name;
            }
        }

        return ""; // 未检测到夹爪
    }

    ModbusGripperBase::ModbusGripperBase(
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        const std::string& joint_name
    )
        : logger_(std::move(logger))
          , clock_(std::move(clock))
          , joint_name_(joint_name)
          , initialized_(false)
    {
    }

    ModbusGripperBase::~ModbusGripperBase()
    {
        // 确保后台线程被停止
        stopBackgroundReading();
    }

    void ModbusGripperBase::exportStateInterfaces(
        std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces
    )
    {
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_name_, hardware_interface::HW_IF_POSITION, getPositionPtr()
            )
        );

        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_name_, hardware_interface::HW_IF_VELOCITY, getVelocityPtr()
            )
        );

        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_name_, hardware_interface::HW_IF_EFFORT, getEffortPtr()
            )
        );
    }

    void ModbusGripperBase::exportCommandInterfaces(
        std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces
    )
    {
        command_interfaces.push_back(
            std::make_shared<hardware_interface::CommandInterface>(
                joint_name_, hardware_interface::HW_IF_POSITION, getPositionCommandPtr()
            )
        );
    }

    void ModbusGripperBase::startBackgroundReading()
    {
        if (reading_thread_active_)
        {
            RCLCPP_WARN(logger_, "Background reading thread is already running");
            return;
        }

        if (!initialized_)
        {
            RCLCPP_ERROR(logger_, "Cannot start background reading: gripper not initialized");
            return;
        }

        reading_thread_stop_ = false;
        reading_thread_active_ = true;

        reading_thread_ = std::thread(&ModbusGripperBase::backgroundReadingLoop, this);

        RCLCPP_INFO(logger_, "Background reading/writing thread started");
    }

    void ModbusGripperBase::stopBackgroundReading()
    {
        if (!reading_thread_active_)
        {
            return;
        }

        reading_thread_stop_ = true;
        reading_thread_active_ = false;

        if (reading_thread_.joinable())
        {
            reading_thread_.join();
        }

        RCLCPP_INFO(logger_, "Background reading thread stopped");
    }

    void ModbusGripperBase::updateBackgroundReadingInterval(const rclcpp::Duration& period)
    {
        // 只设置一次，避免重复更新
        bool expected = false;
        if (!interval_initialized_.compare_exchange_strong(expected, true))
        {
            return; // 已经初始化过了
        }

        // 根据控制循环的周期计算后台线程的循环间隔
        // 后台线程频率设为控制循环频率的 1倍（与控制循环相同频率）
        // 例如：100Hz控制循环（10ms周期）-> 100Hz后台线程（10ms间隔）
        //       50Hz控制循环（20ms周期）-> 50Hz后台线程（20ms间隔）
        double period_sec = period.seconds();
        int calculated_interval_ms = static_cast<int>(period_sec * 1000.0);
        
        // 限制最小间隔为10ms，最大间隔为500ms
        calculated_interval_ms = std::max(10, std::min(500, calculated_interval_ms));
        
        loop_interval_ms_ = calculated_interval_ms;
        
        RCLCPP_INFO(
            logger_,
            "Background reading interval set to %d ms (control period: %.3f s, %.1f Hz, background: %.1f Hz)",
            calculated_interval_ms,
            period_sec,
            1.0 / period_sec,
            1000.0 / calculated_interval_ms
        );
    }

    void ModbusGripperBase::backgroundReadingLoop()
    {
        RCLCPP_DEBUG(logger_, "Background reading/writing loop started");

        while (!reading_thread_stop_ && initialized_)
        {
            auto start_time = std::chrono::steady_clock::now();
            
            // 获取当前的循环间隔（动态更新）
            int current_interval_ms = loop_interval_ms_.load();

            // 首先检查是否有新命令需要写入
            // 使用原子比较：如果 position_command_ 与 last_command_ 不同，说明有新命令
            double current_command = position_command_; // 原子读取
            if (current_command != last_command_)
            {
                // 有新命令，执行写入
                if (writeCommand())
                {
                    RCLCPP_DEBUG_THROTTLE(
                        logger_,
                        *clock_,
                        1000,
                        "Background write command: %.3f",
                        current_command
                    );
                }
                else
                {
                    // 写入失败（可能是命令未变化或其他错误）
                    RCLCPP_DEBUG_THROTTLE(
                        logger_,
                        *clock_,
                        2000,
                        "Background write failed or no change"
                    );
                }
            }

            // 每次循环都读取状态（后台线程机制避免阻塞控制循环）
            if (!readStatus())
            {
                // 读取失败，但不退出循环，继续尝试
                RCLCPP_DEBUG_THROTTLE(
                    logger_,
                    *clock_,
                    2000,
                    "Background read failed, will retry"
                );
            }

            // 计算实际耗时，确保按动态间隔执行
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

            if (elapsed_ms < current_interval_ms)
            {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(current_interval_ms - elapsed_ms)
                );
            }
            else
            {
                // 如果操作耗时超过间隔，记录警告
                RCLCPP_WARN_THROTTLE(
                    logger_,
                    *clock_,
                    5000,
                    "Background loop took %ld ms (expected: %d ms)",
                    elapsed_ms,
                    current_interval_ms
                );
            }
        }

        RCLCPP_DEBUG(logger_, "Background reading/writing loop exited");
    }
} // namespace modbus_ros2_control
