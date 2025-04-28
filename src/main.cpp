#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <vector>
#include <array>
#include <sstream>

class VitalRequester : public rclcpp::Node
{
public:
  VitalRequester()
  : Node("vital_requester")
  {
    // ポートとボーレート
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 2000000);
    port_ = this->get_parameter("port").as_string();
    baud_ = this->get_parameter("baudrate").as_int();

    // シリアルオープン
    if (!openSerial()) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port %s", port_.c_str());
      rclcpp::shutdown();
      return;
    }

    // 対象センサ ID リスト
    const std::vector<uint8_t> ids = {0x0A, 0x0B, 0x0C};

    // 1) 全センサにキャリブレーション要求
    {
      std::ostringstream oss;
      for (auto id : ids) {
        bool ok = sendCalibration(id);
        oss << formatId(id) << ":" << (ok ? "OK" : "FAIL") << "  ";
      }
      RCLCPP_INFO(get_logger(), "Calibration results: %s", oss.str().c_str());
    }

    // 2) 60秒カウントダウン待機
    RCLCPP_INFO(get_logger(), "Waiting 60 seconds for sensors to settle...");
    for (int sec = 60; sec > 0 && rclcpp::ok(); --sec) {
      RCLCPP_INFO(get_logger(), "  Remaining: %2d s", sec);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!rclcpp::ok()) {
      RCLCPP_WARN(get_logger(), "Interrupted during wait");
      rclcpp::shutdown();
      return;
    }

    // 3) 全センサにバイタル値要求
    {
      std::ostringstream oss;
      for (auto id : ids) {
        bool ok = sendVitalRequest(id);
        oss << formatId(id) << ":" << (ok ? "OK" : "FAIL") << "  ";
      }
      RCLCPP_INFO(get_logger(), "Vital request results: %s", oss.str().c_str());
    }

    rclcpp::shutdown();
  }

  ~VitalRequester()
  {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  bool openSerial()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;
    termios tio{};
    cfmakeraw(&tio);
    cfsetispeed(&tio, baud_);
    cfsetospeed(&tio, baud_);
    tio.c_cflag |= CLOCAL | CREAD;
    tcsetattr(fd_, TCSANOW, &tio);
    return true;
  }

  bool sendCalibration(uint8_t id)
  {
    // [AA C1 ID 00 22 55]
    std::array<uint8_t,6> cmd = {{0xAA,0xC1,id,0x00,0x22,0x55}};
    if (::write(fd_, cmd.data(), cmd.size()) != (ssize_t)cmd.size()) {
      return false;
    }
    // ACK: [AA C1 ID 00 00 55]
    std::array<uint8_t,6> resp{};
    ssize_t n = ::read(fd_, resp.data(), resp.size());
    return n == 6
        && resp[0]==0xAA && resp[1]==0xC1
        && resp[2]==id   && resp[3]==0x00
        && resp[4]==0x00 && resp[5]==0x55;
  }

  bool sendVitalRequest(uint8_t id)
  {
    // [AA C1 ID 00 20 55]
    std::array<uint8_t,6> cmd = {{0xAA,0xC1,id,0x00,0x20,0x55}};
    if (::write(fd_, cmd.data(), cmd.size()) != (ssize_t)cmd.size()) {
      return false;
    }
    // 任意長応答を読む
    uint8_t buf[32];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    return n > 0;
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
  std::make_shared<VitalRequester>();
  return 0;
}
