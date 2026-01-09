#pragma once

#include "modbus_ros2_control/hands/dexterous_hand_wrapper_template.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>

namespace modbus_ros2_control
{
    // ============================================================================
    // Helper Functions: Joint Limits from ModbusConfig
    // ============================================================================
    
    /**
     * @brief Get joint limits from ModbusConfig based on joint name and product type
     * @param joint_name Joint name (e.g., "thumb_joint1", "index_joint")
     * @param product_type Product type (O7, O6, or L6)
     * @param lower Output lower limit
     * @param upper Output upper limit
     * @return true if limits found, false otherwise
     */
    inline bool getJointLimitsFromConfig(
        const std::string& joint_name,
        DexterousHandProduct product_type,
        double& lower,
        double& upper
    )
    {
        std::string name_lower = joint_name;
        std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
        
        namespace ModbusConfig = gripper_hardware_common::ModbusConfig;
        
        if (product_type == DexterousHandProduct::O7)
        {
            if (name_lower.find("thumb") != std::string::npos)
            {
                if (name_lower.find("joint1") != std::string::npos)
                {
                    lower = ModbusConfig::DexterousHand::O7::THUMB_JOINT1_LOWER;
                    upper = ModbusConfig::DexterousHand::O7::THUMB_JOINT1_UPPER;
                    return true;
                }
                else if (name_lower.find("joint2") != std::string::npos)
                {
                    lower = ModbusConfig::DexterousHand::O7::THUMB_JOINT2_LOWER;
                    upper = ModbusConfig::DexterousHand::O7::THUMB_JOINT2_UPPER;
                    return true;
                }
                else if (name_lower.find("joint3") != std::string::npos)
                {
                    lower = ModbusConfig::DexterousHand::O7::THUMB_JOINT3_LOWER;
                    upper = ModbusConfig::DexterousHand::O7::THUMB_JOINT3_UPPER;
                    return true;
                }
            }
            else if (name_lower.find("index") != std::string::npos)
            {
                lower = ModbusConfig::DexterousHand::O7::INDEX_JOINT_LOWER;
                upper = ModbusConfig::DexterousHand::O7::INDEX_JOINT_UPPER;
                return true;
            }
            else if (name_lower.find("middle") != std::string::npos)
            {
                lower = ModbusConfig::DexterousHand::O7::MIDDLE_JOINT_LOWER;
                upper = ModbusConfig::DexterousHand::O7::MIDDLE_JOINT_UPPER;
                return true;
            }
            else if (name_lower.find("ring") != std::string::npos)
            {
                lower = ModbusConfig::DexterousHand::O7::RING_JOINT_LOWER;
                upper = ModbusConfig::DexterousHand::O7::RING_JOINT_UPPER;
                return true;
            }
            else if (name_lower.find("pinky") != std::string::npos || name_lower.find("little") != std::string::npos)
            {
                lower = ModbusConfig::DexterousHand::O7::PINKY_JOINT_LOWER;
                upper = ModbusConfig::DexterousHand::O7::PINKY_JOINT_UPPER;
                return true;
            }
        }
        else if (product_type == DexterousHandProduct::O6)
        {
            if (name_lower.find("thumb") != std::string::npos)
            {
                if (name_lower.find("joint1") != std::string::npos)
                {
                    lower = ModbusConfig::DexterousHand::O6::THUMB_JOINT1_LOWER;
                    upper = ModbusConfig::DexterousHand::O6::THUMB_JOINT1_UPPER;
                    return true;
                }
                else if (name_lower.find("joint2") != std::string::npos)
                {
                    lower = ModbusConfig::DexterousHand::O6::THUMB_JOINT2_LOWER;
                    upper = ModbusConfig::DexterousHand::O6::THUMB_JOINT2_UPPER;
                    return true;
                }
            }
            else if (name_lower.find("index") != std::string::npos)
            {
                lower = ModbusConfig::DexterousHand::O6::INDEX_JOINT_LOWER;
                upper = ModbusConfig::DexterousHand::O6::INDEX_JOINT_UPPER;
                return true;
            }
            else if (name_lower.find("middle") != std::string::npos)
            {
                lower = ModbusConfig::DexterousHand::O6::MIDDLE_JOINT_LOWER;
                upper = ModbusConfig::DexterousHand::O6::MIDDLE_JOINT_UPPER;
                return true;
            }
            else if (name_lower.find("ring") != std::string::npos)
            {
                lower = ModbusConfig::DexterousHand::O6::RING_JOINT_LOWER;
                upper = ModbusConfig::DexterousHand::O6::RING_JOINT_UPPER;
                return true;
            }
            else if (name_lower.find("pinky") != std::string::npos || name_lower.find("little") != std::string::npos)
            {
                lower = ModbusConfig::DexterousHand::O6::PINKY_JOINT_LOWER;
                upper = ModbusConfig::DexterousHand::O6::PINKY_JOINT_UPPER;
                return true;
            }
        }
        else if (product_type == DexterousHandProduct::L6)
        {
            if (name_lower.find("thumb") != std::string::npos)
            {
                if (name_lower.find("joint1") != std::string::npos)
                {
                    lower = ModbusConfig::DexterousHand::L6::THUMB_JOINT1_LOWER;
                    upper = ModbusConfig::DexterousHand::L6::THUMB_JOINT1_UPPER;
                    return true;
                }
                else if (name_lower.find("joint2") != std::string::npos)
                {
                    lower = ModbusConfig::DexterousHand::L6::THUMB_JOINT2_LOWER;
                    upper = ModbusConfig::DexterousHand::L6::THUMB_JOINT2_UPPER;
                    return true;
                }
            }
            else if (name_lower.find("index") != std::string::npos)
            {
                lower = ModbusConfig::DexterousHand::L6::INDEX_JOINT_LOWER;
                upper = ModbusConfig::DexterousHand::L6::INDEX_JOINT_UPPER;
                return true;
            }
            else if (name_lower.find("middle") != std::string::npos)
            {
                lower = ModbusConfig::DexterousHand::L6::MIDDLE_JOINT_LOWER;
                upper = ModbusConfig::DexterousHand::L6::MIDDLE_JOINT_UPPER;
                return true;
            }
            else if (name_lower.find("ring") != std::string::npos)
            {
                lower = ModbusConfig::DexterousHand::L6::RING_JOINT_LOWER;
                upper = ModbusConfig::DexterousHand::L6::RING_JOINT_UPPER;
                return true;
            }
            else if (name_lower.find("pinky") != std::string::npos || name_lower.find("little") != std::string::npos)
            {
                lower = ModbusConfig::DexterousHand::L6::PINKY_JOINT_LOWER;
                upper = ModbusConfig::DexterousHand::L6::PINKY_JOINT_UPPER;
                return true;
            }
        }
        
        return false;
    }

    // ============================================================================
    // Traits Implementation: Register Mapping Functions
    // ============================================================================

    // O7 specialization
    inline int DexterousHandProductTraits<DexterousHandProduct::O7>::getModbusRegisterIndex(const std::string& joint_name)
    {
        std::string name_lower = joint_name;
        std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
        
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
        
        return -1;  // Not found
    }

    // O6 specialization
    // Register mapping according to Modbus protocol and URDF definition:
    // Protocol: Address 0=Current_Thumb_Pitch, Address 1=Current_Thumb_Yaw
    // URDF: thumb_joint1 -> Address 0 (Current_Thumb_Pitch), thumb_joint2 -> Address 1 (Current_Thumb_Yaw)
    // Address 0: Current_Thumb_Pitch (thumb_joint1) - 大拇指弯曲
    // Address 1: Current_Thumb_Yaw (thumb_joint2) - 大拇指横摆
    // Address 2: Current_Index_Pitch (index_joint) - 食指弯曲
    // Address 3: Current_Middle_Pitch (middle_joint) - 中指弯曲
    // Address 4: Current_Ring_Pitch (ring_joint) - 无名指弯曲
    // Address 5: Current_Little_Pitch (pinky_joint) - 小拇指弯曲
    inline int DexterousHandProductTraits<DexterousHandProduct::O6>::getModbusRegisterIndex(const std::string& joint_name)
    {
        std::string name_lower = joint_name;
        std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
        
        if (name_lower.find("thumb") != std::string::npos)
        {
            // O6 mapping:
            // Protocol: Address 0=Current_Thumb_Pitch (大拇指弯曲), Address 1=Current_Thumb_Yaw (大拇指横摆)
            // URDF mapping: thumb_joint1 -> Address 1 (Current_Thumb_Yaw), thumb_joint2 -> Address 0 (Current_Thumb_Pitch)
            if (name_lower.find("joint1") != std::string::npos)
            {
                return 1;  // Address 1: Current_Thumb_Yaw (thumb_joint1 -> Address 1)
            }
            else if (name_lower.find("joint2") != std::string::npos)
            {
                return 0;  // Address 0: Current_Thumb_Pitch (thumb_joint2 -> Address 0)
            }
            // Fallback: check for yaw/pitch keywords (for compatibility)
            else if (name_lower.find("pitch") != std::string::npos)
            {
                return 0;  // Address 0: Current_Thumb_Pitch
            }
            else if (name_lower.find("yaw") != std::string::npos)
            {
                return 1;  // Address 1: Current_Thumb_Yaw
            }
            // Note: O6 hand does not have Thumb_Roll (no joint3)
        }
        else if (name_lower.find("index") != std::string::npos)
        {
            return 2;  // Address 2: Current_Index_Pitch (食指弯曲)
        }
        else if (name_lower.find("middle") != std::string::npos)
        {
            return 3;  // Address 3: Current_Middle_Pitch (中指弯曲)
        }
        else if (name_lower.find("ring") != std::string::npos)
        {
            return 4;  // Address 4: Current_Ring_Pitch (无名指弯曲)
        }
        else if (name_lower.find("pinky") != std::string::npos || 
                 name_lower.find("little") != std::string::npos)
        {
            return 5;  // Address 5: Current_Little_Pitch (小拇指弯曲)
        }
        
        return -1;  // Not found
    }

    // L6 specialization
    inline int DexterousHandProductTraits<DexterousHandProduct::L6>::getModbusRegisterIndex(const std::string& joint_name)
    {
        // For now, L6 uses the same mapping as O6
        // Customize this if L6 has a different register layout
        return DexterousHandProductTraits<DexterousHandProduct::O6>::getModbusRegisterIndex(joint_name);
    }

    // ============================================================================
    // Template Class Implementation
    // ============================================================================

    template<DexterousHandProduct Product>
    DexterousHandWrapperTemplate<Product>::DexterousHandWrapperTemplate(
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        const std::vector<std::string>& joint_names
    )
        : DexterousHandBase(std::move(logger), std::move(clock), joint_names)
          , communicator_(nullptr)
          , modbus_slave_id_(0x27)  // Default right hand 0x27
    {
    }

    template<DexterousHandProduct Product>
    DexterousHandWrapperTemplate<Product>::~DexterousHandWrapperTemplate()
    {
        shutdown();
    }

    template<DexterousHandProduct Product>
    bool DexterousHandWrapperTemplate<Product>::initialize(
        ModbusRtuCommunicator* communicator,
        const std::unordered_map<std::string, std::string>& params
    )
    {
        if (!communicator || !communicator->isConnected())
        {
            RCLCPP_ERROR(logger_, "Modbus communicator not connected");
            return false;
        }

        communicator_ = communicator;

        // Load joint limits from ModbusConfig (similar to Jodell RG75)
        RCLCPP_INFO(logger_, "Loading joint limits from ModbusConfig for %s hand", PRODUCT_NAME);
        
        bool all_limits_found = true;
        for (size_t i = 0; i < joint_names_.size() && i < JOINT_COUNT; ++i)
        {
            double lower = 0.0;
            double upper = 0.0;
            
            if (getJointLimitsFromConfig(joint_names_[i], Product, lower, upper))
            {
                joint_lower_limits_[i] = lower;
                joint_upper_limits_[i] = upper;
                RCLCPP_INFO(
                    logger_,
                    "Joint %s: limits [%.4f, %.4f] rad (from ModbusConfig)",
                    joint_names_[i].c_str(),
                    joint_lower_limits_[i],
                    joint_upper_limits_[i]
                );
            }
            else
            {
                RCLCPP_ERROR(
                    logger_,
                    "Joint %s: limits not found in ModbusConfig! Cannot proceed without joint limits.",
                    joint_names_[i].c_str()
                );
                all_limits_found = false;
            }
        }
        
        if (!all_limits_found)
        {
            RCLCPP_ERROR(
                logger_,
                "Failed to load all joint limits from ModbusConfig. Please ensure all %zu joints have limits defined in ModbusConfig.",
                JOINT_COUNT
            );
            return false;
        }
        
        // Print all joint limits
        RCLCPP_INFO(logger_, "=== %s Hand Joint Limits Summary (from ModbusConfig) ===", PRODUCT_NAME);
        RCLCPP_INFO(logger_, "Joint Name          | Modbus Reg | Lower (rad) | Upper (rad) | Range (rad)");
        RCLCPP_INFO(logger_, "--------------------|------------|-------------|-------------|-------------");
        for (size_t i = 0; i < joint_names_.size() && i < JOINT_COUNT; ++i)
        {
            int modbus_idx = Traits::getModbusRegisterIndex(joint_names_[i]);
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

        // Read hand configuration parameter (Modbus ID)
        auto it = params.find("hand_side");
        if (it != params.end())
        {
            std::string side_str = it->second;
            std::transform(side_str.begin(), side_str.end(), side_str.begin(), ::tolower);
            
            if (side_str == "left" || side_str == "left_hand")
            {
                modbus_slave_id_ = DexterousHand::LEFT_HAND_SLAVE_ID;
            }
            else if (side_str == "right" || side_str == "right_hand")
            {
                modbus_slave_id_ = DexterousHand::RIGHT_HAND_SLAVE_ID;
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

        // Read speed limiting configuration
        auto speed_it = params.find("max_speed_ratio");
        if (speed_it != params.end())
        {
            try
            {
                max_speed_ratio_ = std::stod(speed_it->second);
                // Clamp to valid range [0.0, 1.0]
                max_speed_ratio_ = std::max(0.0, std::min(1.0, max_speed_ratio_));
                RCLCPP_INFO(
                    logger_,
                    "Speed limiting enabled: max_speed_ratio = %.2f (%.0f%% of maximum speed)",
                    max_speed_ratio_,
                    max_speed_ratio_ * 100.0
                );
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(
                    logger_,
                    "Failed to parse max_speed_ratio parameter '%s': %s. Using default 1.0 (no speed limiting)",
                    speed_it->second.c_str(),
                    e.what()
                );
                max_speed_ratio_ = 1.0;
            }
        }
        else
        {
            RCLCPP_INFO(logger_, "No max_speed_ratio parameter found, using default 1.0 (no speed limiting)");
        }

        // Read initial position and synchronize command registers
        RCLCPP_INFO(logger_, "Reading initial hand position...");
        if (readStatus())
        {
            // Set command position to current position to avoid jumps
            for (size_t i = 0; i < JOINT_COUNT && i < position_commands_.size(); ++i)
            {
                position_commands_[i] = positions_[i];
            }
            
            // Prepare for initial write with speed limiting support
            const double time_step = 0.01;
            for (size_t i = 0; i < JOINT_COUNT && i < position_commands_.size(); ++i)
            {
                // Set last_commands_ to trigger write while respecting speed limits
                double joint_range = joint_upper_limits_[i] - joint_lower_limits_[i];
                if (joint_range > 0.0 && max_speed_ratio_ < 1.0)
                {
                    double max_change = joint_range * max_speed_ratio_ * time_step;
                    last_commands_[i] = positions_[i] - max_change * 0.5;
                }
                else
                {
                    last_commands_[i] = positions_[i] - 0.0001;
                }
            }
            
            // Write initial command to synchronize device registers
            RCLCPP_INFO(logger_, "Writing initial command to device...");
            writeCommand();  // Ignore return value, continue even if write fails
            
            // Sync last_commands_ with position_commands_ for future writes
            for (size_t i = 0; i < JOINT_COUNT && i < position_commands_.size(); ++i)
            {
                last_commands_[i] = position_commands_[i];
            }
            
            initial_position_read_ = true;
            RCLCPP_INFO(logger_, "Initial position synchronized successfully");
            logInitialized();
        }
        else
        {
            // Use default middle positions if read fails
            RCLCPP_WARN(logger_, "Failed to read initial position, using default middle positions");
            for (size_t i = 0; i < JOINT_COUNT && i < positions_.size(); ++i)
            {
                double middle = (joint_lower_limits_[i] + joint_upper_limits_[i]) / 2.0;
                positions_[i] = middle;
                position_commands_[i] = middle;
                last_commands_[i] = middle - 0.0001;  // Trigger write
            }
            
            writeCommand();  // Write default positions
            
            // Sync last_commands_
            for (size_t i = 0; i < JOINT_COUNT && i < position_commands_.size(); ++i)
            {
                last_commands_[i] = position_commands_[i];
            }
            
            initial_position_read_ = true;
        }

        initialized_ = true;
        return true;
    }

    template<DexterousHandProduct Product>
    bool DexterousHandWrapperTemplate<Product>::readStatus()
    {
        if (!initialized_ || !communicator_)
        {
            return false;
        }

        // Read joint positions using Input Registers (Function Code 04)
        // According to O6 protocol: Only FC 04 (Read Input Registers) and FC 16 (Write Holding Registers) are supported
        // Address 0-5 are Input Registers for current joint position
        uint16_t joint_data[JOINT_COUNT] = {0};
        int rc = communicator_->readInputRegisters(Traits::JOINT_POSITION_REG_START, JOINT_COUNT, joint_data);
        
        if (rc == static_cast<int>(JOINT_COUNT))
        {
            // Create mapping: joint name -> Modbus register index
            std::vector<int> modbus_indices(JOINT_COUNT, -1);
            for (size_t i = 0; i < joint_names_.size() && i < JOINT_COUNT; ++i)
            {
                modbus_indices[i] = Traits::getModbusRegisterIndex(joint_names_[i]);
                if (modbus_indices[i] < 0 || modbus_indices[i] >= static_cast<int>(JOINT_COUNT))
                {
                    RCLCPP_ERROR(logger_, "Failed to map joint %s to Modbus register", joint_names_[i].c_str());
                    return false;
                }
            }
            
            // Successfully read, convert raw values (0-255) to radians
            uint8_t raw_values[JOINT_COUNT];
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                int modbus_idx = modbus_indices[i];
                raw_values[i] = static_cast<uint8_t>(joint_data[modbus_idx] & 0xFF);
                positions_[i] = rawToRadians(i, raw_values[i]);
                velocities_[i] = 0.0;
                efforts_[i] = 0.0;
            }

            logStatus(raw_values);
            return true;
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(
                logger_,
                *clock_,
                2000,
                "Failed to read %s hand position using Function Code 04 (Read Input Registers): %s (read %d registers, expected %zu). "
                "O6 protocol only supports FC 04 (read) and FC 16 (write). "
                "Please check: 1) Modbus slave ID is correct (0x27=right hand, 0x28=left hand), "
                "2) Device is powered on and in Modbus mode, 3) Serial port is correct (/dev/ttyUSB0), "
                "4) Baudrate is 115200, 5) No other process is using the serial port",
                PRODUCT_NAME,
                communicator_->getLastError().c_str(),
                rc,
                JOINT_COUNT
            );
            return false;
        }
    }

    template<DexterousHandProduct Product>
    bool DexterousHandWrapperTemplate<Product>::writeCommand()
    {
        if (!initialized_ || !communicator_)
        {
            return false;
        }

        // Check if command values are valid (not NaN)
        for (size_t i = 0; i < JOINT_COUNT; ++i)
        {
            if (std::isnan(position_commands_[i]))
            {
                RCLCPP_WARN_THROTTLE(
                    logger_,
                    *clock_,
                    2000,
                    "Command value for joint %zu is NaN, skipping write",
                    i
                );
                return false;
            }
        }

        // Apply speed limiting if enabled (max_speed_ratio < 1.0)
        // Use fixed time step (0.01s = 10ms, corresponding to 100Hz control loop)
        // This is a reasonable default for most control loops
        const double time_step = 0.01;  // 10ms time step

        // Apply speed limiting to position commands
        // Note: position_commands_ is std::array<double, 7> in base class, but we only use first JOINT_COUNT elements
        // Create a local array with correct size for this hand type (O7=7, O6/L6=6)
        std::array<double, JOINT_COUNT> limited_commands;
        for (size_t i = 0; i < JOINT_COUNT; ++i)
        {
            limited_commands[i] = position_commands_[i];
        }

        if (max_speed_ratio_ < 1.0)
        {
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                double desired_change = position_commands_[i] - last_commands_[i];
                
                // Calculate maximum allowed change based on joint range and speed ratio
                // Use joint range to estimate reasonable maximum speed
                double joint_range = joint_upper_limits_[i] - joint_lower_limits_[i];
                if (joint_range > 0.0)
                {
                    // Maximum speed: assume full range can be traversed in 1 second at max speed
                    // Then apply speed ratio to limit it
                    double max_speed = joint_range * max_speed_ratio_;  // rad/s (scaled by ratio)
                    double max_change = max_speed * time_step;  // Maximum change per time step
                    
                    // Limit the change
                    if (std::abs(desired_change) > max_change)
                    {
                        limited_commands[i] = last_commands_[i] + std::copysign(max_change, desired_change);
                    }
                }
            }
        }

        // Check if commands have changed (after speed limiting)
        bool has_changes = false;
        for (size_t i = 0; i < JOINT_COUNT; ++i)
        {
            if (std::abs(limited_commands[i] - last_commands_[i]) > 0.001)
            {
                has_changes = true;
                break;
            }
        }

        if (!has_changes)
        {
            return false; // No changes, no need to send
        }

        // Create mapping: joint name -> Modbus register index
        std::vector<int> modbus_indices(JOINT_COUNT, -1);
        for (size_t i = 0; i < joint_names_.size() && i < JOINT_COUNT; ++i)
        {
            modbus_indices[i] = Traits::getModbusRegisterIndex(joint_names_[i]);
            if (modbus_indices[i] < 0 || modbus_indices[i] >= static_cast<int>(JOINT_COUNT))
            {
                RCLCPP_ERROR(logger_, "Failed to map joint %s to Modbus register", joint_names_[i].c_str());
                return false;
            }
            // Debug: log mapping for O6/L6 hands
            if (JOINT_COUNT == 6)
            {
                RCLCPP_DEBUG(logger_, "Joint %s (index %zu) -> Modbus register %d", 
                            joint_names_[i].c_str(), i, modbus_indices[i]);
            }
        }
        
        // Convert radians commands to raw values (0-255), arranged in Modbus register order
        // Use speed-limited commands instead of original position_commands_
        uint16_t raw_positions[JOINT_COUNT] = {0};
        for (size_t i = 0; i < JOINT_COUNT; ++i)
        {
            int modbus_idx = modbus_indices[i];
            raw_positions[modbus_idx] = static_cast<uint16_t>(radiansToRaw(i, limited_commands[i]));
        }

        // Use Modbus function code 16 to write multiple holding registers
        int rc = communicator_->writeRegisters(Traits::JOINT_POSITION_REG_START, JOINT_COUNT, raw_positions);
        
        if (rc == static_cast<int>(JOINT_COUNT))
        {
            // Update last_commands_ with speed-limited commands
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                last_commands_[i] = limited_commands[i];
            }
            
            logCommand(raw_positions);
            return true;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *clock_,
                1000,
                "Failed to write %s hand command: %s (wrote %d registers, expected %zu)",
                PRODUCT_NAME,
                communicator_->getLastError().c_str(),
                rc,
                JOINT_COUNT
            );
            return false;
        }
    }

    template<DexterousHandProduct Product>
    void DexterousHandWrapperTemplate<Product>::shutdown()
    {
        if (initialized_)
        {
            stopBackgroundReading();
            communicator_ = nullptr;
            initialized_ = false;
        }
    }

    template<DexterousHandProduct Product>
    uint8_t DexterousHandWrapperTemplate<Product>::radiansToRaw(size_t joint_index, double radians) const
    {
        if (joint_index >= JOINT_COUNT)
        {
            return 0;
        }

        double lower = joint_lower_limits_[joint_index];
        double upper = joint_upper_limits_[joint_index];
        double range = upper - lower;
        
        if (range <= 0.0)
        {
            return 128;
        }

        double normalized = (radians - lower) / range;
        normalized = std::max(0.0, std::min(1.0, normalized));
        
        return static_cast<uint8_t>(255 - std::round(normalized * 255.0));
    }

    template<DexterousHandProduct Product>
    double DexterousHandWrapperTemplate<Product>::rawToRadians(size_t joint_index, uint8_t raw) const
    {
        if (joint_index >= JOINT_COUNT)
        {
            return 0.0;
        }

        double lower = joint_lower_limits_[joint_index];
        double upper = joint_upper_limits_[joint_index];
        double range = upper - lower;
        
        if (range <= 0.0)
        {
            return (lower + upper) / 2.0;
        }

        double normalized = static_cast<double>(255 - raw) / 255.0;
        return lower + normalized * range;
    }

    template<DexterousHandProduct Product>
    void DexterousHandWrapperTemplate<Product>::logInitialized()
    {
        std::stringstream ss;
        ss << PRODUCT_NAME << " Dexterous hand initialized, Modbus ID: 0x" 
           << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(modbus_slave_id_)
           << std::dec << ", initial positions: [";
        
        for (size_t i = 0; i < JOINT_COUNT; ++i)
        {
            ss << std::fixed << std::setprecision(4) << positions_[i];
            if (i < JOINT_COUNT - 1) ss << ", ";
        }
        ss << "] rad";
        
        RCLCPP_INFO(logger_, "%s", ss.str().c_str());
    }

    template<DexterousHandProduct Product>
    void DexterousHandWrapperTemplate<Product>::logStatus(const uint8_t* raw_values)
    {
        RCLCPP_INFO_THROTTLE(
            logger_,
            *clock_,
            1000,
            "=== %s Hand Status ===",
            PRODUCT_NAME
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
        for (size_t i = 0; i < JOINT_COUNT; ++i)
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
        
        // Log position array
        std::stringstream pos_ss, raw_ss;
        pos_ss << "Current positions (rad): [";
        raw_ss << "Modbus raw values:       [";
        
        for (size_t i = 0; i < JOINT_COUNT; ++i)
        {
            pos_ss << std::fixed << std::setprecision(4) << positions_[i];
            raw_ss << std::setw(3) << static_cast<int>(raw_values[i]);
            if (i < JOINT_COUNT - 1)
            {
                pos_ss << ", ";
                raw_ss << ", ";
            }
        }
        pos_ss << "]";
        raw_ss << "]";
        
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "%s", pos_ss.str().c_str());
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "%s", raw_ss.str().c_str());
    }

    template<DexterousHandProduct Product>
    void DexterousHandWrapperTemplate<Product>::logCommand(const uint16_t* raw_positions)
    {
        RCLCPP_INFO_THROTTLE(
            logger_,
            *clock_,
            1000,
            "=== %s Hand Command ===",
            PRODUCT_NAME
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
        for (size_t i = 0; i < JOINT_COUNT; ++i)
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
        
        // Log command array
        std::stringstream cmd_ss, raw_ss;
        cmd_ss << "Command positions (rad): [";
        raw_ss << "Modbus raw values:     [";
        
        for (size_t i = 0; i < JOINT_COUNT; ++i)
        {
            cmd_ss << std::fixed << std::setprecision(4) << position_commands_[i];
            raw_ss << std::setw(3) << static_cast<int>(raw_positions[i]);
            if (i < JOINT_COUNT - 1)
            {
                cmd_ss << ", ";
                raw_ss << ", ";
            }
        }
        cmd_ss << "]";
        raw_ss << "]";
        
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "%s", cmd_ss.str().c_str());
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "%s", raw_ss.str().c_str());
    }
} // namespace modbus_ros2_control

