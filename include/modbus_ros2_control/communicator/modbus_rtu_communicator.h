#pragma once

#include <modbus/modbus.h>
#include <string>

namespace modbus_ros2_control {

/**
 * @brief Modbus RTU 通信封装类
 * 
 * 封装 libmodbus 的 RTU 通信功能，提供统一的 Modbus 读写接口
 */
class ModbusRtuCommunicator {
public:
    /**
     * @brief 构造函数
     * @param serial_port 串口路径（如 "/dev/ttyUSB0"）
     * @param baudrate 波特率（如 115200）
     * @param slave_id 从站地址（1-247）
     * @param parity 校验位（'N'=无校验, 'E'=偶校验, 'O'=奇校验）
     * @param data_bits 数据位（通常为 8）
     * @param stop_bits 停止位（通常为 1）
     */
    ModbusRtuCommunicator(
        const std::string& serial_port,
        uint32_t baudrate,
        int slave_id,
        char parity = 'N',
        int data_bits = 8,
        int stop_bits = 1
    );

    /**
     * @brief 析构函数，自动关闭连接
     */
    ~ModbusRtuCommunicator();

    /**
     * @brief 初始化并连接 Modbus 设备
     * @return 是否成功
     */
    bool connect();

    /**
     * @brief 断开 Modbus 连接
     */
    void disconnect();

    /**
     * @brief 检查是否已连接
     * @return 是否已连接
     */
    bool isConnected() const { return modbus_ctx_ != nullptr; }

    /**
     * @brief 读取保持寄存器
     * @param addr 起始寄存器地址
     * @param count 读取的寄存器数量
     * @param dest 目标缓冲区（至少 count 个 uint16_t）
     * @return 成功读取的寄存器数量，失败返回 -1
     */
    int readHoldingRegisters(uint16_t addr, int count, uint16_t* dest);

    /**
     * @brief 写入单个保持寄存器
     * @param addr 寄存器地址
     * @param value 要写入的值
     * @return 是否成功
     */
    bool writeRegister(uint16_t addr, uint16_t value);

    /**
     * @brief 写入多个保持寄存器
     * @param addr 起始寄存器地址
     * @param count 写入的寄存器数量
     * @param src 源数据缓冲区
     * @return 成功写入的寄存器数量，失败返回 -1
     */
    int writeRegisters(uint16_t addr, int count, const uint16_t* src);

    /**
     * @brief 设置调试模式
     * @param debug 是否启用调试（1=启用，0=禁用）
     */
    void setDebug(bool debug);

    /**
     * @brief 获取最后的错误信息
     * @return 错误信息字符串
     */
    std::string getLastError() const;

private:
    std::string serial_port_;
    uint32_t baudrate_;
    int slave_id_;
    char parity_;
    int data_bits_;
    int stop_bits_;
    
    modbus_t* modbus_ctx_;
    bool connected_;
};

} // namespace modbus_ros2_control

