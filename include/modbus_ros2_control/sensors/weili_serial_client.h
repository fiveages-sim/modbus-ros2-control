#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace modbus_ros2_control
{

/**
 * @brief Weili 六维力传感器低层串口客户端（Modbus RTU，通用协议 1）。
 *
 * 机器人/上位机作为 Modbus 主站，传感器为从站：
 *   - 串口参数：8N1，默认波特率 115200（USB-RS485 转换器默认值；机器人内部 485 总线可为 921600），从站号 9。
 *   - 命令（FC 0x10 写单寄存器 0x019A，回传/启动/停止/回零见协议说明）。
 *   - 传感器回传数据帧（帧头 0x20 0x4E + 6 个 float32 + CRC16/Modbus 低位在前），
 *     以及回零状态 / 错误状态短帧。
 *
 * 与 `kwr75_serial_client` 一样使用原生 termios 串口（不依赖 libmodbus），
 * 命令帧只发出、不等待 Modbus ACK（与天机 485 / Jodell RG75 行为一致）。
 */
class WeiliSerialClient
{
public:
  static constexpr std::size_t kAxisCount = 6;   ///< Fx Fy Fz Mx My Mz
  static constexpr std::size_t kCrcBytes = 2;
  static constexpr std::size_t kMaxDataBytes = kAxisCount * 4;  ///< 24 字节力值
  static constexpr std::size_t kDefaultFrameLength = 28;        ///< 头2 + 力值24 + CRC2
  static constexpr std::size_t kMaxFrameLength = 29;            ///< 若含功能位则 +1
  static constexpr std::size_t kShortFrameLength = 6;           ///< 回零状态 / 错误状态短帧

  // Modbus RTU 命令常量
  static constexpr uint8_t kDefaultSlaveId = 0x09;      ///< 从站号 9
  static constexpr uint8_t kFunctionCode = 0x10;        ///< FC 0x10：写多个寄存器
  static constexpr uint16_t kControlRegister = 0x019A;  ///< 控制寄存器地址
  static constexpr uint16_t kStopValue = 0x0300;        ///< 停止数据传输
  static constexpr uint16_t kRate250Hz = 0x0301;        ///< 250 Hz
  static constexpr uint16_t kRate500Hz = 0x0302;        ///< 500 Hz
  static constexpr uint16_t kRate1000Hz = 0x0303;       ///< 1000 Hz
  static constexpr uint16_t kZeroValue = 0x8051;        ///< 回零（依据文档 CRC 0x966C 反推）

  // 回传帧
  static constexpr uint8_t kFrameHeader1 = 0x20;
  static constexpr uint8_t kFrameHeader2 = 0x4E;
  static constexpr uint8_t kErrorTag = 0x01;            ///< 错误状态帧功能位
  static constexpr uint8_t kZeroStatusTag = 0x00;       ///< 回零状态帧功能位

  /**
   * @brief 构造。
   * @param serial_port 串口路径（如 /dev/ttyUSB0）
   * @param baudrate 波特率（默认 115200，8N1）
   * @param slave_id Modbus 从站号（默认 9）
   * @param start_value 启动数据回传的寄存器值（默认 0x0301 = 250 Hz）
   * @param data_offset 力值在帧中的起始偏移（默认 2：头2 + 24力值 + CRC2）
   * @param frame_length 完整力值帧长度（默认 28；若实际含功能位字节则为 29）
   */
  WeiliSerialClient(
    std::string serial_port,
    int baudrate = 115200,
    uint8_t slave_id = kDefaultSlaveId,
    uint16_t start_value = kRate250Hz,
    std::size_t data_offset = 2,
    std::size_t frame_length = kDefaultFrameLength,
    int zero_timeout_ms = 500,
    int read_timeout_ms = 4,
    int startup_delay_ms = 100,
    int warmup_attempts = 20);

  bool connect();
  void disconnect();
  bool is_connected() const { return serial_fd_ >= 0; }

  /** 发送启动指令（只发一次），并准备接收数据流。 */
  bool start_streaming();
  /** 发送停止数据传输指令（0x0300）。 */
  bool stop_streaming();

  /**
   * @brief 请求回零并等待回零结果（协议：500 ms 超时）。
   * @param status_out 回零状态（0 正常，非 0 回零失败）
   * @return true 表示在超时内收到回零结果且状态为 0；
   *         超时 / 状态非 0 / 通讯失败均返回 false（调用方可用 status_out 区分）。
   */
  bool request_zero(uint8_t & status_out);

  /**
   * @brief 读取最新一帧六维力数据（采样保持）。
   *
   * 非阻塞优先：若缓冲里已有完整新帧立即解析；否则按需等待不超过 wait_ms。
   * 返回值 only 表示“本次是否解析到新帧”，数据总是通过 out 回填（无新帧时回填最近一帧）。
   * 帧头不符 / CRC 校验失败的包会被丢弃。
   *
   * @return true 表示解析到新帧（out 为最新值）。
   */
  bool read_wrench(
    std::array<double, kAxisCount> & out,
    int wait_ms = 0);

  /** 距上一次成功解析到新帧的时间（用于数据中断看门狗）。 */
  long long millis_since_last_data() const;

  /** 传感器错误状态帧是否曾被收到（读后由上层负责清除语义）。 */
  bool has_error_report() const { return has_error_report_.load(); }
  uint8_t last_error_code() const { return last_error_code_; }
  void clear_error_report() { has_error_report_.store(false); }

  /** 最近串口字节 hex（诊断用）。 */
  std::string last_io_sample_hex() const { return last_io_sample_hex_; }

  const std::string & serial_port() const { return serial_port_; }
  bool has_valid_sample() const { return has_valid_sample_; }
  std::size_t frame_length() const { return frame_length_; }
  std::size_t data_offset() const { return data_offset_; }

private:
  bool configure_serial();
  bool send_command(uint16_t register_value);
  bool read_into_buffer(std::chrono::milliseconds wait);
  /** 从缓冲头部逐个消费完整帧，返回缓冲内最新一帧力值并刷新采样保持。 */
  bool consume_latest_force_frame();
  bool try_parse_force_frame(std::size_t start, std::array<float, kAxisCount> & values) const;
  bool try_parse_short_frame(std::size_t start, uint8_t & tag, uint8_t & code) const;
  void record_io_sample(const std::vector<uint8_t> & buffer);
  static float decode_wire_float(const uint8_t * wire_bytes);
  static uint16_t crc16_modbus(const uint8_t * data, std::size_t len);

  std::string serial_port_;
  int baudrate_;
  uint8_t slave_id_;
  uint16_t start_value_;
  std::size_t data_offset_;
  std::size_t frame_length_;
  int zero_timeout_ms_;
  int read_timeout_ms_;
  int startup_delay_ms_;
  int warmup_attempts_;

  bool streaming_started_ = false;
  int serial_fd_ = -1;
  std::vector<uint8_t> rx_buffer_;
  std::array<float, kAxisCount> latest_raw_{};
  std::array<double, kAxisCount> latest_wrench_{};
  std::chrono::steady_clock::time_point last_data_time_{};
  bool has_valid_sample_ = false;

  std::atomic<bool> has_error_report_{false};
  uint8_t last_error_code_ = 0;
  std::string last_io_sample_hex_;
};

}  // namespace modbus_ros2_control
