#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace modbus_ros2_control
{

/** Low-level KWR75 RS485 client (28-byte float frame, default 115200 8N1). */
class Kwr75SerialClient
{
public:
  static constexpr std::size_t kAxisCount = 6;
  static constexpr std::size_t kFrameLength = 28;

  Kwr75SerialClient(
    std::string serial_port,
    int baudrate = 115200,
    uint8_t command_code = 0x48,
    bool convert_to_si = true,
    double gravity = 9.80665,
    int response_timeout_ms = 50,
    int read_timeout_ms = 3,
    int startup_delay_ms = 50,
    int warmup_attempts = 20);

  bool connect();
  void disconnect();
  bool is_connected() const { return serial_fd_ >= 0; }
  bool warmup();
  /** timeout_ms < 0 用 read_timeout_ms_；=0 表示「立刻取最新缓存帧，不等」。 */
  bool read_wrench(std::array<double, kAxisCount>& wrench_si, int timeout_ms = -1);
  /** Hex dump of recent serial bytes when warmup/read fails (for diagnostics). */
  std::string last_io_sample_hex() const
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    return last_io_sample_hex_;
  }
  const std::string& serial_port() const { return serial_port_; }

private:
  bool configure_serial();
  bool start_capture();
  void stop_reader();
  bool send_start_command();
  /** 后台线程：持续排空串口，把最新完整帧缓存起来。 */
  void reader_loop();
  bool read_latest_frame(std::array<uint8_t, kFrameLength>& frame, int timeout_ms);
  void record_io_sample(const std::vector<uint8_t>& buffer);
  static float decode_wire_float(const uint8_t* wire_bytes);
  static bool parse_frame(
    const std::array<uint8_t, kFrameLength>& frame,
    std::array<float, kAxisCount>& values,
    uint8_t command_code);

  std::string serial_port_;
  int baudrate_;
  uint8_t command_code_;
  bool convert_to_si_;
  double gravity_;
  int response_timeout_ms_;
  int read_timeout_ms_;
  int startup_delay_ms_;
  int warmup_attempts_;
  bool streaming_started_ = false;
  int serial_fd_ = -1;
  mutable std::mutex io_mutex_;
  std::string last_io_sample_hex_;

  // ── 后台收帧线程 ────────────────────────────────────────────────────────
  // 115200 8N1 下 28 字节帧的线上时间就是 2.43 ms，比 500 Hz 控制周期
  // (2 ms) 还长。所以在实时环里同步 read() 必然 Overrun。这里把串口读取
  // 完全搬到独立线程，read() 只做一次加锁 memcpy（~µs）。
  std::thread reader_thread_;
  std::atomic<bool> reader_running_ {false};
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::array<uint8_t, kFrameLength> latest_frame_ {};
  std::chrono::steady_clock::time_point latest_frame_stamp_ {};
  bool have_frame_ = false;
  uint64_t frame_seq_ = 0;
  /** 超过这个时间没有新帧，read_wrench() 才报失败（避免单帧丢失就置 0）。 */
  int stale_timeout_ms_ = 50;
};

}  // namespace modbus_ros2_control
