#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include <cerrno>
#include <cstring>

namespace modbus_ros2_control {

ModbusRtuCommunicator::ModbusRtuCommunicator(
    const std::string& serial_port,
    uint32_t baudrate,
    int slave_id,
    char parity,
    int data_bits,
    int stop_bits
)
    : serial_port_(serial_port)
    , baudrate_(baudrate)
    , slave_id_(slave_id)
    , parity_(parity)
    , data_bits_(data_bits)
    , stop_bits_(stop_bits)
    , modbus_ctx_(nullptr)
    , connected_(false)
{
}

ModbusRtuCommunicator::~ModbusRtuCommunicator() {
    disconnect();
}

bool ModbusRtuCommunicator::connect() {
    if (connected_) {
        return true;
    }

    // 创建 Modbus RTU 上下文
    modbus_ctx_ = modbus_new_rtu(
        serial_port_.c_str(),
        static_cast<int>(baudrate_),
        parity_,
        data_bits_,
        stop_bits_
    );

    if (!modbus_ctx_) {
        return false;
    }

    // 设置调试模式（默认关闭）
    modbus_set_debug(modbus_ctx_, 0);

    // 设置从站地址
    if (modbus_set_slave(modbus_ctx_, slave_id_) == -1) {
        modbus_free(modbus_ctx_);
        modbus_ctx_ = nullptr;
        return false;
    }

    // 连接设备
    if (modbus_connect(modbus_ctx_) == -1) {
        modbus_free(modbus_ctx_);
        modbus_ctx_ = nullptr;
        return false;
    }

    connected_ = true;
    return true;
}

void ModbusRtuCommunicator::disconnect() {
    if (modbus_ctx_) {
        modbus_close(modbus_ctx_);
        modbus_free(modbus_ctx_);
        modbus_ctx_ = nullptr;
    }
    connected_ = false;
}

int ModbusRtuCommunicator::readHoldingRegisters(uint16_t addr, int count, uint16_t* dest) {
    if (!connected_ || !modbus_ctx_) {
        return -1;
    }

    return modbus_read_registers(modbus_ctx_, addr, count, dest);
}

bool ModbusRtuCommunicator::writeRegister(uint16_t addr, uint16_t value) {
    if (!connected_ || !modbus_ctx_) {
        return false;
    }

    return modbus_write_register(modbus_ctx_, addr, value) == 1;
}

int ModbusRtuCommunicator::writeRegisters(uint16_t addr, int count, const uint16_t* src) {
    if (!connected_ || !modbus_ctx_) {
        return -1;
    }

    return modbus_write_registers(modbus_ctx_, addr, count, src);
}

void ModbusRtuCommunicator::setDebug(bool debug) {
    if (modbus_ctx_) {
        modbus_set_debug(modbus_ctx_, debug ? 1 : 0);
    }
}

std::string ModbusRtuCommunicator::getLastError() const {
    if (modbus_ctx_) {
        return modbus_strerror(errno);
    }
    return "Modbus context not initialized";
}

} // namespace modbus_ros2_control

