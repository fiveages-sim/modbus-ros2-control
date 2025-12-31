#include "modbus_ros2_control/hands/simple_dexterous_hand_wrapper.h"
#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include "arms_controller_common/utils/JointLimitsManager.h"
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <vector>
#include <climits>
#include <cstddef>

using namespace gripper_hardware_common;
using ModbusConfig::DexterousHand;

namespace modbus_ros2_control
{
    // 位置转换函数：弧度 <-> Modbus 原始值 (0-255)
    uint8_t SimpleDexterousHandWrapper::radiansToRaw(size_t joint_index, double radians) const
    {
        if (joint_index >= 7)
        {
            return 0;
        }

        // 将弧度值限制在关节范围内
        double lower = joint_lower_limits_[joint_index];
        double upper = joint_upper_limits_[joint_index];
        double range = upper - lower;
        
        if (range <= 0.0)
        {
            // 如果范围无效，返回中间值
            return 128;
        }

        // 将弧度值归一化到 [0, 1]
        double normalized = (radians - lower) / range;
        normalized = std::max(0.0, std::min(1.0, normalized));
        
        // 映射到 0-255
        return static_cast<uint8_t>(255 - std::round(normalized * 255.0));
    }

    double SimpleDexterousHandWrapper::rawToRadians(size_t joint_index, uint8_t raw) const
    {
        if (joint_index >= 7)
        {
            return 0.0;
        }

        double lower = joint_lower_limits_[joint_index];
        double upper = joint_upper_limits_[joint_index];
        double range = upper - lower;
        
        if (range <= 0.0)
        {
            // 如果范围无效，返回中间值
            return (lower + upper) / 2.0;
        }

        // 将 0-255 归一化到 [0, 1]
        double normalized = static_cast<double>(255 - raw) / 255.0;
        
        // 映射到弧度范围
        return lower + normalized * range;
    }

    int SimpleDexterousHandWrapper::getModbusRegisterIndex(const std::string& joint_name) const
    {
        std::string name_lower = joint_name;
        std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
        
        // Modbus 寄存器映射（根据协议定义）
        // 寄存器 0: Thumb_Pitch (thumb_joint1)
        // 寄存器 1: Thumb_Yaw (thumb_joint2)
        // 寄存器 2: Index_Pitch (index_joint)
        // 寄存器 3: Middle_Pitch (middle_joint)
        // 寄存器 4: Ring_Pitch (ring_joint)
        // 寄存器 5: Little_Pitch (pinky_joint)
        // 寄存器 6: Thumb_Roll (thumb_joint3)
        
        if (name_lower.find("thumb") != std::string::npos)
        {
            if (name_lower.find("joint1") != std::string::npos)
            {
                return 6;  // Thumb_Roll (joint1 实际对应 Roll)
            }
            else if (name_lower.find("joint2") != std::string::npos || 
                     name_lower.find("yaw") != std::string::npos)
            {
                return 1;  // Thumb_Yaw
            }
            else if (name_lower.find("joint3") != std::string::npos || 
                     name_lower.find("roll") != std::string::npos)
            {
                return 0;  // Thumb_Pitch (joint3 实际对应 Pitch)
            }
            else if (name_lower.find("pitch") != std::string::npos)
            {
                return 0;  // Thumb_Pitch
            }
        }
        else if (name_lower.find("index") != std::string::npos)
        {
            return 2;  // Index_Pitch
        }
        else if (name_lower.find("middle") != std::string::npos)
        {
            return 3;  // Middle_Pitch
        }
        else if (name_lower.find("ring") != std::string::npos)
        {
            return 4;  // Ring_Pitch
        }
        else if (name_lower.find("pinky") != std::string::npos || 
                 name_lower.find("little") != std::string::npos)
        {
            return 5;  // Little_Pitch
        }
        
        return -1;  // 未找到匹配
    }

    SimpleDexterousHandWrapper::SimpleDexterousHandWrapper(
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        const std::vector<std::string>& joint_names
    )
        : DexterousHandBase(std::move(logger), std::move(clock), joint_names)
          , communicator_(nullptr)
          , modbus_slave_id_(0x27)  // 默认右手 0x27
    {
    }

    SimpleDexterousHandWrapper::~SimpleDexterousHandWrapper()
    {
        shutdown();
    }

    bool SimpleDexterousHandWrapper::initialize(
        ModbusRtuCommunicator* communicator,
        const std::unordered_map<std::string, std::string>& params,
        const std::string& robot_description
    )
    {
        if (!communicator || !communicator->isConnected())
        {
            RCLCPP_ERROR(logger_, "Modbus communicator not connected");
            return false;
        }

        communicator_ = communicator;

        // 从 URDF 中解析关节限制
        if (!robot_description.empty())
        {
            arms_controller_common::JointLimitsManager limits_manager(logger_);
            size_t parsed_count = limits_manager.parseFromURDF(robot_description, joint_names_);
            
            if (parsed_count > 0)
            {
                RCLCPP_INFO(logger_, "Parsed joint limits for %zu/%zu joints from URDF", parsed_count, joint_names_.size());
                
                // 读取每个关节的限制（必须从 URDF 中读取，不使用默认值）
                bool all_limits_found = true;
                for (size_t i = 0; i < joint_names_.size() && i < 7; ++i)
                {
                    auto limits = limits_manager.getJointLimits(joint_names_[i]);
                    if (limits.initialized)
                    {
                        joint_lower_limits_[i] = limits.lower;
                        joint_upper_limits_[i] = limits.upper;
                        RCLCPP_INFO(
                            logger_,
                            "Joint %s: limits [%.4f, %.4f] rad (from URDF)",
                            joint_names_[i].c_str(),
                            joint_lower_limits_[i],
                            joint_upper_limits_[i]
                        );
                    }
                    else
                    {
                        RCLCPP_ERROR(
                            logger_,
                            "Joint %s: limits not found in URDF! Cannot proceed without joint limits.",
                            joint_names_[i].c_str()
                        );
                        all_limits_found = false;
                    }
                }
                
                if (!all_limits_found)
                {
                    RCLCPP_ERROR(
                        logger_,
                        "Failed to load all joint limits from URDF. Please ensure all 7 joints have <limit> tags in the URDF."
                    );
                    return false;
                }
                
                // 打印所有关节的限制值
                RCLCPP_INFO(logger_, "=== Joint Limits Summary (from URDF) ===");
                RCLCPP_INFO(logger_, "Joint Name          | Modbus Reg | Lower (rad) | Upper (rad) | Range (rad)");
                RCLCPP_INFO(logger_, "--------------------|------------|-------------|-------------|-------------");
                for (size_t i = 0; i < joint_names_.size() && i < 7; ++i)
                {
                    int modbus_idx = getModbusRegisterIndex(joint_names_[i]);
                    double range = joint_upper_limits_[i] - joint_lower_limits_[i];
                    RCLCPP_INFO(
                        logger_,
                        "%-18s | %10d | %11.4f | %11.4f | %11.4f",
                        joint_names_[i].c_str(),
                        modbus_idx,
                        joint_lower_limits_[i],
                        joint_upper_limits_[i],
                        range
                    );
                }
                RCLCPP_INFO(logger_, "==========================================");
                
                // 注意：关节限制直接从 URDF 读取，不需要交换
                // thumb_joint1 映射到寄存器 6 (Thumb_Roll)，使用 URDF 中的限制
                // thumb_joint3 映射到寄存器 0 (Thumb_Pitch)，使用 URDF 中的限制 (0.5146)
                // URDF 中的限制值已经是正确的物理限制，直接使用即可
            }
            else
            {
                RCLCPP_ERROR(
                    logger_,
                    "Failed to parse joint limits from URDF. Cannot proceed without joint limits. Please ensure all 7 joints have <limit> tags in the URDF."
                );
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(
                logger_,
                "robot_description is empty. Cannot load joint limits from URDF. Please ensure robot_description parameter is set."
            );
            return false;
        }

        // 读取手部配置参数（Modbus ID）
        auto it = params.find("hand_side");
        if (it != params.end())
        {
            std::string side_str = it->second;
            std::transform(side_str.begin(), side_str.end(), side_str.begin(), ::tolower);
            
            if (side_str == "left" || side_str == "left_hand")
            {
                modbus_slave_id_ = DexterousHand::LEFT_HAND_SLAVE_ID;  // 左手 Modbus ID
            }
            else if (side_str == "right" || side_str == "right_hand")
            {
                modbus_slave_id_ = DexterousHand::RIGHT_HAND_SLAVE_ID;  // 右手 Modbus ID
            }
            else
            {
                RCLCPP_WARN(
                    logger_,
                    "Unknown hand_side parameter: %s, using default RIGHT_HAND (0x27)",
                    it->second.c_str()
                );
            }
        }

        // 设置 Modbus 从站地址
        // 注意：ModbusRtuCommunicator 在连接时已设置从站地址
        // 如果需要动态切换，需要重新连接

        // 读取初始位置
        if (readStatus())
        {
            // 将当前位置设置为命令位置
            position_commands_ = positions_;
            last_commands_ = positions_;
            RCLCPP_INFO(
                logger_,
                "Dexterous hand initialized, Modbus ID: 0x%02X, initial positions: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f] rad",
                modbus_slave_id_,
                positions_[0], positions_[1], positions_[2], positions_[3],
                positions_[4], positions_[5], positions_[6]
            );
        }
        else
        {
            RCLCPP_WARN(logger_, "Failed to read initial hand position, using default middle positions");
            // 设置默认位置为中间值（每个关节的中间弧度值）
            for (size_t i = 0; i < 7; ++i)
            {
                double middle = (joint_lower_limits_[i] + joint_upper_limits_[i]) / 2.0;
                positions_[i] = middle;
                position_commands_[i] = middle;
                last_commands_[i] = middle;
            }
        }

        initialized_ = true;
        return true;
    }

    bool SimpleDexterousHandWrapper::readStatus()
    {
        if (!initialized_ || !communicator_)
        {
            return false;
        }

        // 使用 Modbus 功能码 04 读取输入寄存器（根据协议，O7机械手支持功能码 04/16）
        // 根据协议，关节位置存储在寄存器 0-6
        // 每个关节使用1个寄存器，值为 0-255
        
        // 读取7个关节的位置（从寄存器地址 JOINT_POSITION_REG_START = 0 开始）
        // 注意：协议要求使用功能码 04 (读输入寄存器)，而不是 03 (读保持寄存器)
        uint16_t joint_data[DexterousHand::JOINT_COUNT] = {0};
        int rc = communicator_->readInputRegisters(DexterousHand::JOINT_POSITION_REG_START, DexterousHand::JOINT_COUNT, joint_data);
        
        if (rc == DexterousHand::JOINT_COUNT)
        {
            // 创建映射：关节名称 -> Modbus 寄存器索引
            std::vector<int> modbus_indices(7, -1);
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                modbus_indices[i] = getModbusRegisterIndex(joint_names_[i]);
                if (modbus_indices[i] < 0 || modbus_indices[i] >= 7)
                {
                    RCLCPP_ERROR(logger_, "Failed to map joint %s to Modbus register", joint_names_[i].c_str());
                    return false;
                }
            }
            
            // 成功读取，转换原始值 (0-255) 为弧度值
            // 注意：需要根据映射表从正确的 Modbus 寄存器读取
            uint8_t raw_values[DexterousHand::JOINT_COUNT];
            for (size_t i = 0; i < DexterousHand::JOINT_COUNT; ++i)
            {
                // 根据映射表找到对应的 Modbus 寄存器索引
                int modbus_idx = modbus_indices[i];
                raw_values[i] = static_cast<uint8_t>(joint_data[modbus_idx] & 0xFF);
                positions_[i] = rawToRadians(i, raw_values[i]);
                // velocities_ 和 efforts_ 暂时设为0，可以从其他寄存器读取
                velocities_[i] = 0.0;
                efforts_[i] = 0.0;
            }

            // 打印 Modbus 原始值和转换后的关节值
            RCLCPP_INFO_THROTTLE(
                logger_,
                *clock_,
                1000,
                ""
            );
            RCLCPP_INFO_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Joint Name          | Modbus Raw | Joint Value (rad) | Limits [lower, upper]"
            );
            RCLCPP_INFO_THROTTLE(
                logger_,
                *clock_,
                1000,
                "--------------------|------------|--------------------|----------------------"
            );
            for (size_t i = 0; i < DexterousHand::JOINT_COUNT; ++i)
            {
                RCLCPP_INFO_THROTTLE(
                    logger_,
                    *clock_,
                    1000,
                    "%-18s | %10d | %18.4f | [%.4f, %.4f]",
                    joint_names_[i].c_str(),
                    static_cast<int>(raw_values[i]),
                    positions_[i],
                    joint_lower_limits_[i],
                    joint_upper_limits_[i]
                );
            }
            RCLCPP_INFO_THROTTLE(
                logger_,
                *clock_,
                1000,
                "=========================================="
            );

            // 添加一行格式化的数组输出
            RCLCPP_INFO_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Current positions (rad): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                positions_[0], positions_[1], positions_[2],
                positions_[3], positions_[4], positions_[5],
                positions_[6]
            );
            RCLCPP_INFO_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Modbus raw values:       [%3d, %3d, %3d, %3d, %3d, %3d, %3d]",
                static_cast<int>(raw_values[0]),
                static_cast<int>(raw_values[1]),
                static_cast<int>(raw_values[2]),
                static_cast<int>(raw_values[3]),
                static_cast<int>(raw_values[4]),
                static_cast<int>(raw_values[5]),
                static_cast<int>(raw_values[6])
            );

            RCLCPP_DEBUG_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Reading dexterous hand status: positions=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f] rad (raw=[%d, %d, %d, %d, %d, %d, %d])",
                positions_[0], positions_[1], positions_[2], positions_[3],
                positions_[4], positions_[5], positions_[6],
                static_cast<int>(raw_values[0]),
                static_cast<int>(raw_values[1]),
                static_cast<int>(raw_values[2]),
                static_cast<int>(raw_values[3]),
                static_cast<int>(raw_values[4]),
                static_cast<int>(raw_values[5]),
                static_cast<int>(raw_values[6])
            );

            return true;
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Failed to read dexterous hand position: %s (read %d registers, expected %d)",
                communicator_->getLastError().c_str(),
                rc,
                DexterousHand::JOINT_COUNT
            );
            return false;
        }
    }

    bool SimpleDexterousHandWrapper::writeCommand()
    {
        if (!initialized_ || !communicator_)
        {
            return false;
        }

        // 检查是否有命令变化
        bool has_changes = false;
        for (size_t i = 0; i < DexterousHand::JOINT_COUNT; ++i)
        {
            if (std::abs(position_commands_[i] - last_commands_[i]) > 0.001)
            {
                has_changes = true;
                break;
            }
        }

        if (!has_changes)
        {
            return false; // 命令未变化，无需发送
        }

        // 创建映射：关节名称 -> Modbus 寄存器索引
        std::vector<int> modbus_indices(7, -1);
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            modbus_indices[i] = getModbusRegisterIndex(joint_names_[i]);
            if (modbus_indices[i] < 0 || modbus_indices[i] >= 7)
            {
                RCLCPP_ERROR(logger_, "Failed to map joint %s to Modbus register", joint_names_[i].c_str());
                return false;
            }
        }
        
        // 转换弧度命令为原始值 (0-255)，并按照 Modbus 寄存器顺序排列
        // 根据协议：小值弯曲，大值伸直（对于弯曲关节）
        uint16_t raw_positions[DexterousHand::JOINT_COUNT] = {0};
        for (size_t i = 0; i < DexterousHand::JOINT_COUNT; ++i)
        {
            int modbus_idx = modbus_indices[i];
            raw_positions[modbus_idx] = static_cast<uint16_t>(radiansToRaw(i, position_commands_[i]));
        }

        // 使用 Modbus 功能码 16 写入多个保持寄存器
        // 写入7个关节的位置（从寄存器地址 JOINT_POSITION_REG_START = 0 开始）
        // 寄存器映射：
        // 0: Thumb_Pitch (大拇指弯曲)
        // 1: Thumb_Yaw (大拇指横摆)
        // 2: Index_Pitch (食指弯曲)
        // 3: Middle_Pitch (中指弯曲)
        // 4: Ring_Pitch (无名指弯曲)
        // 5: Little_Pitch (小拇指弯曲)
        // 6: Thumb_Roll (大拇指横滚)
        int rc = communicator_->writeRegisters(DexterousHand::JOINT_POSITION_REG_START, DexterousHand::JOINT_COUNT, raw_positions);
        
        if (rc == DexterousHand::JOINT_COUNT)
        {
            last_commands_ = position_commands_;
            
            // 打印命令关节值和 Modbus 原始值
            RCLCPP_INFO_THROTTLE(
                logger_,
                *clock_,
                1000,
                "=== Dexterous Hand Command (movej) ==="
            );
            RCLCPP_INFO_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Joint Name          | Command (rad) | Modbus Raw | Limits [lower, upper]"
            );
            RCLCPP_INFO_THROTTLE(
                logger_,
                *clock_,
                1000,
                "--------------------|----------------|------------|----------------------"
            );
            for (size_t i = 0; i < DexterousHand::JOINT_COUNT; ++i)
            {
                RCLCPP_INFO_THROTTLE(
                    logger_,
                    *clock_,
                    1000,
                    "%-18s | %14.4f | %10d | [%.4f, %.4f]",
                    joint_names_[i].c_str(),
                    position_commands_[i],
                    static_cast<int>(raw_positions[i]),
                    joint_lower_limits_[i],
                    joint_upper_limits_[i]
                );
            }
            RCLCPP_INFO_THROTTLE(
                logger_,
                *clock_,
                1000,
                "=========================================="
            );
            
            // 也打印一行格式化的数组，方便复制
            RCLCPP_INFO_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Command positions (rad): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                position_commands_[0], position_commands_[1], position_commands_[2],
                position_commands_[3], position_commands_[4], position_commands_[5],
                position_commands_[6]
            );
            RCLCPP_INFO_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Modbus raw values:     [%3d, %3d, %3d, %3d, %3d, %3d, %3d]",
                static_cast<int>(raw_positions[0]),
                static_cast<int>(raw_positions[1]),
                static_cast<int>(raw_positions[2]),
                static_cast<int>(raw_positions[3]),
                static_cast<int>(raw_positions[4]),
                static_cast<int>(raw_positions[5]),
                static_cast<int>(raw_positions[6])
            );
            
            RCLCPP_DEBUG_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Writing dexterous hand command: positions=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f] rad (raw=[%d, %d, %d, %d, %d, %d, %d])",
                position_commands_[0], position_commands_[1], position_commands_[2],
                position_commands_[3], position_commands_[4], position_commands_[5],
                position_commands_[6],
                static_cast<int>(raw_positions[0]),
                static_cast<int>(raw_positions[1]),
                static_cast<int>(raw_positions[2]),
                static_cast<int>(raw_positions[3]),
                static_cast<int>(raw_positions[4]),
                static_cast<int>(raw_positions[5]),
                static_cast<int>(raw_positions[6])
            );
            
            return true;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Failed to write dexterous hand command: %s (wrote %d registers, expected %d)",
                communicator_->getLastError().c_str(),
                rc,
                DexterousHand::JOINT_COUNT
            );
            return false;
        }
    }

    void SimpleDexterousHandWrapper::shutdown()
    {
        if (initialized_)
        {
            // 停止后台读取线程
            stopBackgroundReading();

            communicator_ = nullptr;
            initialized_ = false;
        }
    }
} // namespace modbus_ros2_control

