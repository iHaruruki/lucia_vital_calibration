#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>

class VitalCalibrator : public rclcpp::Node
{
public:
  VitalCalibrator()
  : Node("vital_calibrator")
  {
    // パラメータ読み取り
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 2000000);
    port_ = this->get_parameter("port").as_string();
    baud_ = this->get_parameter("baudrate").as_int();

    // シリアルポートオープン
    if (!openSerial()) {
      RCLCPP_FATAL(this->get_logger(), "Cannot open serial port %s", port_.c_str());
      return;
    }

    // １．キャリブレーション要求
    auto cal_summary = calibrateAll();
    RCLCPP_INFO(this->get_logger(), "Calibration summary: %s", cal_summary.c_str());

    if (!rclcpp::ok()) {
      RCLCPP_WARN(this->get_logger(), "Interrupted after calibration");
      return;
    }

    // ２．カウントダウン表示しつつ 60秒待機
    RCLCPP_INFO(this->get_logger(), "Waiting 60 seconds for sensors to settle...");
    for (int sec = 60; sec > 0 && rclcpp::ok(); --sec) {
      RCLCPP_INFO(this->get_logger(), "  Remaining: %2d seconds", sec);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (!rclcpp::ok()) {
      RCLCPP_WARN(this->get_logger(), "Interrupted during wait");
      return;
    }

    // ３．バイタル値要求
    auto vital_summary = requestVitalsAll();
    RCLCPP_INFO(this->get_logger(), "Vital request summary: %s", vital_summary.c_str());
  }

  ~VitalCalibrator()
  {
    if (fd_ >= 0) close(fd_);
  }

private:
  bool openSerial()
  {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;
    termios tio{};
    cfmakeraw(&tio);
    cfsetispeed(&tio, B2000000);
    cfsetospeed(&tio, B2000000);
    tio.c_cflag |= CLOCAL | CREAD;
    tcsetattr(fd_, TCSANOW, &tio);
    return true;
  }

  std::string calibrateAll()
  {
    const std::vector<uint8_t> ids = {0x0A, 0x0B, 0x0C};
    std::ostringstream summary;
    for (auto id : ids) {
      if (!rclcpp::ok()) break;
      bool ok = sendResetCommand(id);
      summary << formatId(id) << ":" << (ok ? "OK" : "FAIL") << "; ";
    }
    return summary.str();
  }

  std::string requestVitalsAll()
  {
    const std::vector<uint8_t> ids = {0x0A, 0x0B, 0x0C};
    std::ostringstream summary;
    for (auto id : ids) {
      if (!rclcpp::ok()) break;
      bool ok = sendVitalRequest(id);
      summary << formatId(id) << ":" << (ok ? "OK" : "FAIL") << "; ";
    }
    return summary.str();
  }

  bool sendResetCommand(uint8_t id)
  {
    uint8_t cmd[5] = {0xAA, 0xC1, id, 0x22, 0x55};
    if (write(fd_, cmd, sizeof(cmd)) != sizeof(cmd)) return false;
    uint8_t buf[5];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    return (n == sizeof(buf)
         && buf[0]==0xAA && buf[1]==0xC1
         && buf[2]==id   && buf[3]==0x00
         && buf[4]==0x55);
  }

  bool sendVitalRequest(uint8_t id)
  {
    uint8_t cmd[5] = {0xC1, id, 0x00, 0x20, 0x55};
    if (write(fd_, cmd, sizeof(cmd)) != sizeof(cmd)) return false;
    uint8_t buf[32];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    return (n > 0);
  }

  std::string formatId(uint8_t id)
  {
    std::ostringstream ss;
    ss << "0x"
       << std::hex << std::uppercase
       << std::setw(2) << std::setfill('0')
       << static_cast<int>(id);
    return ss.str();
  }

  std::string port_;
  int baud_;
  int fd_{-1};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // signal handler は rclcpp::init で設定されるので、
  // ループ中に rclcpp::ok() をチェックするだけで Ctrl+C が効きます。
  auto node = std::make_shared<VitalCalibrator>();
  rclcpp::shutdown();
  return 0;
}
