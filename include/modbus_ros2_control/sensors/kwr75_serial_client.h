#pragma once

#include <array>
#include <atomic>
#include <chrono>
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

  // ── 失效诊断（只读）────────────────────────────────────────────────────
  /** 距最近一次成功解析帧的毫秒数；从未收到过帧时返回 -1。 */
  long last_frame_age_ms() const;
  uint64_t frames_received() const { return frames_received_.load(std::memory_order_relaxed); }
  bool reader_running() const { return reader_running_.load(std::memory_order_relaxed); }
  /** reader 线程最近一次退出的原因（空 = 仍在运行或从未启动）。 */
  std::string reader_stop_reason() const;

private:
  bool configure_serial();
  bool start_capture();
  void stop_reader();
  bool send_start_command();
  /** 后台线程：持续排空串口，把最新完整帧缓存起来。 */
  void reader_loop();
  bool acquire_latest_frame(std::array<uint8_t, kFrameLength>& frame, int timeout_ms);
  void record_io_sample(const std::vector<uint8_t>& buffer);
  void set_reader_stop_state(const std::string& reason);
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

  /**
   * 无锁帧交接（seqlock）。写者只有 reader 线程、读者只有控制线程：
   * 写者更新期间 try_read() 有界自旋后直接返回 false（绝不阻塞等待），
   * 实时环不会被非实时线程的调度延迟传染；28 字节经 s1==s2 校验不会撕裂。
   */
  struct FrameSlot
  {
    alignas(64) std::atomic<uint64_t> sequence{0};
    std::array<uint8_t, kFrameLength> frame{};
    std::atomic<long long> stamp_ns{0};
    std::atomic<bool> has_data{false};

    void reset();
    void publish(const std::array<uint8_t, kFrameLength>& src, long long now_ns);
    bool try_read(std::array<uint8_t, kFrameLength>& out, long long& stamp_ns_out) const;
  };

  // ── 后台收帧线程 ────────────────────────────────────────────────────────
  // 115200 8N1 下 28 字节帧的线上时间就是 2.43 ms，比 500 Hz 控制周期
  // (2 ms) 还长。所以在实时环里同步 read() 必然 Overrun。这里把串口读取
  // 完全搬到独立线程，read() 只做一次无锁 memcpy（~µs）。
  std::thread reader_thread_;
  std::atomic<bool> reader_running_ {false};
  FrameSlot latest_frame_;
  /** 仅控制线程访问：最近一次成功取到的帧快照（seqlock 竞争失败时复用）。 */
  std::array<uint8_t, kFrameLength> last_delivered_frame_ {};
  long long last_delivered_stamp_ns_ {-1};
  bool last_delivered_valid_ {false};
  /** 超过这个时间没有新帧，read_wrench() 才报失败（避免单帧丢失就置 0）。 */
  int stale_timeout_ms_ = 50;

  // ── 诊断统计（原子量，reader 线程与 read() 并发访问）──────────────────
  std::atomic<uint64_t> frames_received_ {0};
  // reader 线程退出现场（io_mutex_ 保护）
  std::string reader_stop_reason_;  // 空 = 仍在运行或从未启动
};

}  // namespace modbus_ros2_control
