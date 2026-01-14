#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include <cerrno>
#include <cstring>
#include <sys/time.h>

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
        // modbus_new_rtu 失败时，errno 可能没有设置，使用更通用的错误信息
        char error_msg[512];
        const char* modbus_error = (errno != 0) ? modbus_strerror(errno) : nullptr;
        if (modbus_error && strlen(modbus_error) > 0) {
            snprintf(error_msg, sizeof(error_msg), 
                     "Failed to create Modbus RTU context for %s (baudrate=%u, parity=%c, data_bits=%d, stop_bits=%d). Error: %s (errno=%d)",
                     serial_port_.c_str(), baudrate_, parity_, data_bits_, stop_bits_,
                     modbus_error, errno);
        } else {
            snprintf(error_msg, sizeof(error_msg),
                     "Failed to create Modbus RTU context for %s (baudrate=%u, parity=%c, data_bits=%d, stop_bits=%d). Possible causes: invalid parameters, libmodbus initialization failure, or serial port access denied",
                     serial_port_.c_str(), baudrate_, parity_, data_bits_, stop_bits_);
        }
        last_error_ = error_msg;
        return false;
    }

    // 设置调试模式（默认关闭）
    modbus_set_debug(modbus_ctx_, 0);

    // 设置超时时间（响应超时和字节超时）
    // 响应超时：500ms（等待响应的最大时间）
    // 字节超时：100ms（两个字节之间的最大间隔时间）
    struct timeval response_timeout;
    response_timeout.tv_sec = 0;
    response_timeout.tv_usec = 500000; // 500ms
    modbus_set_response_timeout(modbus_ctx_, response_timeout.tv_sec, response_timeout.tv_usec);

    struct timeval byte_timeout;
    byte_timeout.tv_sec = 0;
    byte_timeout.tv_usec = 100000; // 100ms
    modbus_set_byte_timeout(modbus_ctx_, byte_timeout.tv_sec, byte_timeout.tv_usec);

    // 设置从站地址
    if (modbus_set_slave(modbus_ctx_, slave_id_) == -1) {
        last_error_ = "Failed to set slave ID " + std::to_string(slave_id_) + ": " + std::string(modbus_strerror(errno));
        modbus_free(modbus_ctx_);
        modbus_ctx_ = nullptr;
        return false;
    }

    // 连接设备
    if (modbus_connect(modbus_ctx_) == -1) {
        last_error_ = "Failed to connect to " + serial_port_ + ": " + std::string(modbus_strerror(errno));
        modbus_free(modbus_ctx_);
        modbus_ctx_ = nullptr;
        return false;
    }

    connected_ = true;
    last_error_.clear();
    return true;
}

void ModbusRtuCommunicator::disconnect() {
    if (modbus_ctx_) {
        modbus_close(modbus_ctx_);
        modbus_free(modbus_ctx_);
        modbus_ctx_ = nullptr;
    }
    connected_ = false;
    last_error_.clear();
}

int ModbusRtuCommunicator::readHoldingRegisters(uint16_t addr, int count, uint16_t* dest) {
    if (!connected_ || !modbus_ctx_) {
        return -1;
    }

    return modbus_read_registers(modbus_ctx_, addr, count, dest);
}

int ModbusRtuCommunicator::readInputRegisters(uint16_t addr, int count, uint16_t* dest) {
    if (!connected_ || !modbus_ctx_) {
        return -1;
    }

    return modbus_read_input_registers(modbus_ctx_, addr, count, dest);
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
    if (!last_error_.empty()) {
        return last_error_;
    }
    if (modbus_ctx_) {
        return modbus_strerror(errno);
    }
    return "Modbus context not initialized";
}

} // namespace modbus_ros2_control

