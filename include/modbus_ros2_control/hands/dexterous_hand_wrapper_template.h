#pragma once

#include "modbus_ros2_control/hands/dexterous_hand_base.h"
#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include <unordered_map>
#include <memory>
#include <array>
#include <string>

using namespace gripper_hardware_common;
namespace DexterousHand = ModbusConfig::DexterousHand;

namespace modbus_ros2_control
{
    /**
     * @brief Product type enum for different dexterous hand models
     */
    enum class DexterousHandProduct
    {
        O7,  // 7-DOF hand
        O6,  // 6-DOF hand
        L6   // 6-DOF hand (different model)
    };

    /**
     * @brief Traits class for dexterous hand products
     * 
     * Specialize this for different hand products to define:
     * - Joint count (DOF)
     * - Register mapping
     * - Product name for logging
     */
    template<DexterousHandProduct Product>
    struct DexterousHandProductTraits;

    /**
     * @brief O7 hand traits (7-DOF)
     */
    template<>
    struct DexterousHandProductTraits<DexterousHandProduct::O7>
    {
        static constexpr size_t JOINT_COUNT = 7;
        /** FC16 写保持寄存器 / FC04 读输入寄存器：位置 0-6 */
        static constexpr uint16_t JOINT_POSITION_REG_START = 0;
        /** 力矩 7-13（O7 协议） */
        static constexpr uint16_t JOINT_TORQUE_REG_START = 7;
        /** 速度 14-20（O7 协议） */
        static constexpr uint16_t JOINT_SPEED_REG_START = 14;
        static constexpr const char* PRODUCT_NAME = "O7";

        /**
         * @brief Map joint name to Modbus register index for O7 hand
         *
         * FC16/FC04 关节顺序（O7 协议 1.3）：
         * 0 Thumb_Pitch, 1 Thumb_Yaw, 2 Index, 3 Middle, 4 Ring, 5 Little, 6 Thumb_Roll
         * URDF: thumb_joint1=Pitch, thumb_joint2=Yaw, thumb_joint3=Roll
         */
        static int getModbusRegisterIndex(const std::string& joint_name);
    };

    /**
     * @brief O6 hand traits (6-DOF)
     * 
     * Modbus protocol mapping (Function Code 04: Read Input Registers):
     * Address 0-5: Current Joint Position (Read-Only Input Registers)
     * Address 0-5: Target Joint Position (Write Holding Registers, Function Code 16)
     */
    template<>
    struct DexterousHandProductTraits<DexterousHandProduct::O6>
    {
        static constexpr size_t JOINT_COUNT = 6;
        /** FC04/FC16：位置 0-5 */
        static constexpr uint16_t JOINT_POSITION_REG_START = 0;
        /** 力矩 6-11（O6 协议，紧接位置区） */
        static constexpr uint16_t JOINT_TORQUE_REG_START = 6;
        /** 速度 12-17（O6 协议） */
        static constexpr uint16_t JOINT_SPEED_REG_START = 12;
        static constexpr const char* PRODUCT_NAME = "O6";
        
        /**
         * @brief Map joint name to Modbus register index for O6 hand
         * 
         * Register mapping (according to Modbus protocol and URDF definition):
         * Protocol: Address 0=Current_Thumb_Pitch, Address 1=Current_Thumb_Yaw
         * URDF mapping: thumb_joint1 (第一个关节) -> Address 0 (Current_Thumb_Pitch - 大拇指弯曲)
         *               thumb_joint2 (第二个关节) -> Address 1 (Current_Thumb_Yaw - 大拇指横摆)
         * 
         * Address 0: Current_Thumb_Pitch (thumb_joint1) - 大拇指弯曲
         * Address 1: Current_Thumb_Yaw (thumb_joint2) - 大拇指横摆
         * Address 2: Current_Index_Pitch (index_joint) - 食指弯曲
         * Address 3: Current_Middle_Pitch (middle_joint) - 中指弯曲
         * Address 4: Current_Ring_Pitch (ring_joint) - 无名指弯曲
         * Address 5: Current_Little_Pitch (pinky_joint) - 小拇指弯曲
         * 
         * Value range: 0-255 (0 = bend/close, 255 = straight/open)
         * Read: Function Code 04 (Read Input Registers)
         * Write: Function Code 16 (Write Multiple Holding Registers)
         */
        static int getModbusRegisterIndex(const std::string& joint_name);
    };

    /**
     * @brief L6 hand traits (6-DOF)
     * 
     * L6 is a different 6-DOF hand model. If it has the same register mapping as O6,
     * you can reuse O6's mapping. Otherwise, customize it here.
     */
    template<>
    struct DexterousHandProductTraits<DexterousHandProduct::L6>
    {
        static constexpr size_t JOINT_COUNT = 6;
        static constexpr uint16_t JOINT_POSITION_REG_START = 0;
        static constexpr uint16_t JOINT_TORQUE_REG_START = 6;
        static constexpr uint16_t JOINT_SPEED_REG_START = 12;
        static constexpr const char* PRODUCT_NAME = "L6";
        
        /**
         * @brief Map joint name to Modbus register index for L6 hand
         * 
         * Register mapping (customize if different from O6):
         * 0: Thumb_Pitch
         * 1: Thumb_Yaw
         * 2: Index_Pitch
         * 3: Middle_Pitch
         * 4: Ring_Pitch
         * 5: Little_Pitch
         * 
         * Note: Currently using same mapping as O6. Modify if L6 has different mapping.
         */
        static int getModbusRegisterIndex(const std::string& joint_name);
    };

    /**
     * @brief Template-based dexterous hand wrapper
     * 
     * Common implementation for all dexterous hand products (O7, O6, L6, etc.).
     * Configuration is specified as a template parameter from DexterousHandProduct.
     * 
     * @tparam Product Product type (O7, O6, L6, etc.)
     */
    template<DexterousHandProduct Product>
    class DexterousHandWrapperTemplate : public DexterousHandBase
    {
    public:
        using Traits = DexterousHandProductTraits<Product>;
        static constexpr size_t JOINT_COUNT = Traits::JOINT_COUNT;
        static constexpr const char* PRODUCT_NAME = Traits::PRODUCT_NAME;

        /**
         * @brief Constructor
         */
        DexterousHandWrapperTemplate(
            rclcpp::Logger logger,
            rclcpp::Clock::SharedPtr clock,
            const std::vector<std::string>& joint_names
        );

        /**
         * @brief Destructor
         */
        ~DexterousHandWrapperTemplate() override;

        /**
         * @brief Initialize the dexterous hand
         */
        bool initialize(
            ModbusRtuCommunicator* communicator,
            const std::unordered_map<std::string, std::string>& params
        ) override;

        /**
         * @brief Read hand status
         */
        bool readStatus() override;

        /**
         * @brief Write hand command
         */
        bool writeCommand() override;

        /**
         * @brief Shutdown hand connection
         */
        void shutdown() override;

    private:
        // Modbus communicator
        ModbusRtuCommunicator* communicator_;
        
        // Hand configuration
        uint8_t modbus_slave_id_;

        // Joint limits (radians)
        std::array<double, JOINT_COUNT> joint_lower_limits_ = {};
        std::array<double, JOINT_COUNT> joint_upper_limits_ = {};
        
        // Speed limiting configuration
        double max_speed_ratio_ = 1.0;  // Maximum speed ratio (0.0-1.0), limits the maximum speed portion of each joint

        // Position conversion: ROS2 Control radians <-> Modbus raw value (0x00-0xFF)
        uint8_t radiansToRaw(size_t joint_index, double radians) const;
        double rawToRadians(size_t joint_index, uint8_t raw) const;

        // Logging helpers
        void logInitialized();
        void logStatus(const uint8_t* raw_values);
        void logCommand(const uint16_t* raw_positions);
    };

    // Type aliases for convenience
    using SimpleDexterousHandWrapper = DexterousHandWrapperTemplate<DexterousHandProduct::O7>;  // O7 hand (7-DOF)
    using O6DexterousHandWrapper = DexterousHandWrapperTemplate<DexterousHandProduct::O6>;      // O6 hand (6-DOF)
    using L6DexterousHandWrapper = DexterousHandWrapperTemplate<DexterousHandProduct::L6>;      // L6 hand (6-DOF)
} // namespace modbus_ros2_control

// Include template implementation
#include "modbus_ros2_control/hands/linkerhand/dexterous_hand_wrapper_template_impl.h"
