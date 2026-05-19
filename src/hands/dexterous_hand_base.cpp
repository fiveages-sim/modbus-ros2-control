#include "modbus_ros2_control/hands/dexterous_hand_base.h"
#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include <algorithm>
#include <cctype>
#include <limits>
#include <utility>
#include <chrono>
#include <thread>

namespace modbus_ros2_control
{
    std::vector<std::string> DexterousHandBase::detectHandJoints(
        const std::vector<hardware_interface::ComponentInfo>& joints
    )
    {
        std::vector<std::string> hand_joints;
        
        // 灵巧手通常有7个关节，名称可能包含：thumb, index, middle, ring, little, pinky, finger, hand
        // 或者使用数字编号：hand_joint_0, hand_joint_1, etc.
        const std::vector<std::string> hand_keywords = {
            "thumb", "index", "middle", "ring", "little", "pinky", "finger", "hand"
        };
        
        for (const auto& joint : joints)
        {
            std::string joint_name_lower = joint.name;
            std::transform(
                joint_name_lower.begin(),
                joint_name_lower.end(),
                joint_name_lower.begin(),
                ::tolower
            );

            // 检查是否包含手部关键词
            for (const auto& keyword : hand_keywords)
            {
                if (joint_name_lower.find(keyword) != std::string::npos)
                {
                    hand_joints.push_back(joint.name);
                    break;
                }
            }
        }
        for(int i =0; i < hand_joints.size(); i++)
        {
            std::cout << "Detected hand joint: %s" << hand_joints[i] << std::endl;
        }
        // 如果找到的关节数量正好是7个（O7）或6个（O6），返回它们
        if (hand_joints.size() == 7 || hand_joints.size() == 6)
        {
            return hand_joints;
        }

        // 如果找到的关节数量不是7个或6个，返回空列表（未检测到完整的灵巧手）
        // 注意：这里返回空列表会导致错误，但这是预期的行为
        return {}; // 未检测到完整的灵巧手（需要7个或6个关节）
    }

    DexterousHandBase::DexterousHandBase(
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        const std::vector<std::string>& joint_names
    )
        : logger_(std::move(logger))
          , clock_(std::move(clock))
          , joint_names_(joint_names)
          , initialized_(false)
    {
        if (joint_names.size() != 7 && joint_names.size() != 6)
        {
            RCLCPP_WARN(
                logger_,
                "DexterousHandBase expects 7 (O7) or 6 (O6) joints, got %zu",
                joint_names.size()
            );
        }
        last_effort_applied_.fill(std::numeric_limits<double>::quiet_NaN());
        last_velocity_applied_.fill(std::numeric_limits<double>::quiet_NaN());
    }

    DexterousHandBase::~DexterousHandBase()
    {
        // 确保后台线程被停止
        stopBackgroundReading();
    }

    void DexterousHandBase::exportStateInterfaces(
        std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces
    )
    {
        for (size_t i = 0; i < joint_names_.size() && i < 7; ++i)
        {
            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i], hardware_interface::HW_IF_POSITION, getPositionPtr(i)
                )
            );
        }
    }

    void DexterousHandBase::exportCommandInterfaces(
        std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces
    )
    {
        for (size_t i = 0; i < joint_names_.size() && i < 7; ++i)
        {
            command_interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint_names_[i], hardware_interface::HW_IF_POSITION, getPositionCommandPtr(i)
                )
            );
        }
    }

    void DexterousHandBase::applyToolDynamics(const double torque, const double velocity)
    {
        const double t = std::clamp(torque, 0.0, 1.0);
        const double v = std::clamp(velocity, 0.0, 1.0);
        const size_t n = std::min(joint_names_.size(), static_cast<size_t>(7));
        for (size_t i = 0; i < n; ++i)
        {
            effort_commands_[i] = t;
            velocity_commands_[i] = v;
            last_effort_applied_[i] = std::numeric_limits<double>::quiet_NaN();
            last_velocity_applied_[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }

    void DexterousHandBase::startBackgroundReading()
    {
        if (reading_thread_active_)
        {
            RCLCPP_WARN(logger_, "Background reading thread is already running");
            return;
        }

        if (!initialized_)
        {
            RCLCPP_ERROR(logger_, "Cannot start background reading: hand not initialized");
            return;
        }

        reading_thread_stop_ = false;
        reading_thread_active_ = true;
        reading_thread_ = std::thread(&DexterousHandBase::backgroundReadingLoop, this);

        RCLCPP_INFO(logger_, "Background reading thread started for dexterous hand");
    }

    void DexterousHandBase::stopBackgroundReading()
    {
        if (!reading_thread_active_)
        {
            return;
        }

        reading_thread_stop_ = true;
        if (reading_thread_.joinable())
        {
            reading_thread_.join();
        }
        reading_thread_active_ = false;

        RCLCPP_INFO(logger_, "Background reading thread stopped for dexterous hand");
    }

    void DexterousHandBase::updateBackgroundReadingInterval(const rclcpp::Duration& period)
    {
        if (interval_initialized_)
        {
            return; // 只设置一次
        }

        // 将周期转换为毫秒
        int period_ms = static_cast<int>(period.seconds() * 1000.0);
        
        // 确保间隔至少为10ms
        if (period_ms < 10)
        {
            period_ms = 10;
        }

        loop_interval_ms_ = period_ms;
        interval_initialized_ = true;

        RCLCPP_INFO(
            logger_,
            "Background reading thread configured: reading hand joint status every %d ms (%.1f Hz)",
            period_ms,
            1000.0 / period_ms
        );
    }

    void DexterousHandBase::backgroundReadingLoop()
    {
        RCLCPP_INFO(logger_, "Background reading loop started - periodically reading hand joint status");

        while (!reading_thread_stop_)
        {
            auto loop_start = std::chrono::steady_clock::now();

            // 定时读取手关节状态
            if (initialized_)
            {
                readStatus();
                // After first successful read, mark initial position as read
                if (!initial_position_read_.load())
                {
                    initial_position_read_ = true;
                    RCLCPP_INFO(logger_, "Initial position read in background thread. Commands will now be written.");
                }
            }

            // 写入命令 - 只有在已读取初始位置后才写入，避免启动时跳变
            if (initialized_ && initial_position_read_.load())
            {
                writeCommand();
            }
            else if (initialized_ && !initial_position_read_.load())
            {
                // Still waiting for initial position read
                RCLCPP_DEBUG_THROTTLE(
                    logger_,
                    *clock_,
                    1000,
                    "Waiting for initial position read before writing commands..."
                );
            }

            // 计算循环时间并休眠
            auto loop_end = std::chrono::steady_clock::now();
            auto loop_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                loop_end - loop_start
            ).count();

            int sleep_ms = loop_interval_ms_ - static_cast<int>(loop_elapsed);
            if (sleep_ms > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            }
            else
            {
                // 循环时间超过预期，记录警告
                RCLCPP_WARN_THROTTLE(
                    logger_,
                    *clock_,
                    5000,
                    "Background reading loop is slower than expected: %ld ms (expected: %d ms)",
                    loop_elapsed,
                    loop_interval_ms_.load()
                );
            }
        }

        RCLCPP_INFO(logger_, "Background reading loop stopped");
    }
} // namespace modbus_ros2_control

