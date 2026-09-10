#include "modbus_ros2_control/hands/dexterous_hand_base.h"
#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include <algorithm>
#include <cctype>
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
        for (const auto& joint_name : hand_joints)
        {
            std::cout << "Detected hand joint: %s" << joint_name << std::endl;
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

            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i], hardware_interface::HW_IF_VELOCITY, getVelocityPtr(i)
                )
            );

            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i], hardware_interface::HW_IF_EFFORT, getEffortPtr(i)
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
        const int64_t period_ns = period.nanoseconds();
        if (period_ns <= 0) return;

        communication_period_ns_.store(period_ns);
        if (interval_initialized_.exchange(true)) return;

        RCLCPP_INFO(
            logger_,
            "Background communication period configured: %.3f ms (%.1f Hz)",
            period_ns / 1000000.0,
            1.0e9 / static_cast<double>(period_ns)
        );
    }

    void DexterousHandBase::publishCommands()
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        communication_commands_ = position_commands_;
        ++command_sequence_;
    }

    void DexterousHandBase::publishFeedbackToStateInterfaces()
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        positions_ = communication_positions_;
    }

    void DexterousHandBase::backgroundReadingLoop()
    {
        RCLCPP_INFO(logger_, "Background reading loop started - periodically reading hand joint status");

        while (!reading_thread_stop_)
        {
            auto loop_start = std::chrono::steady_clock::now();

            // One thread owns the RTU bus. A changed command is written before feedback
            // is requested; stale intermediate CM commands are intentionally overwritten.
            if (initialized_ && initial_position_read_.load()) writeCommand();
            if (initialized_ && readStatus()) initial_position_read_ = true;

            // 计算循环时间并休眠
            const auto period = std::chrono::nanoseconds(communication_period_ns_.load());
            const auto deadline = loop_start + period;
            const auto loop_end = std::chrono::steady_clock::now();
            if (deadline > loop_end) std::this_thread::sleep_until(deadline);
            else
            {
                // 循环时间超过预期，记录警告
                RCLCPP_WARN_THROTTLE(
                    logger_,
                    *clock_,
                    5000,
                    "Background communication loop missed its %.3f ms period (elapsed %.3f ms)",
                    std::chrono::duration<double, std::milli>(period).count(),
                    std::chrono::duration<double, std::milli>(loop_end - loop_start).count()
                );
            }
        }

        RCLCPP_INFO(logger_, "Background reading loop stopped");
    }
} // namespace modbus_ros2_control
