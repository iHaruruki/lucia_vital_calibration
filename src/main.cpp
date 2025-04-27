#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <iomanip>
#include <sstream>

using Empty  = std_msgs::msg::Empty;
using String = std_msgs::msg::String;

class VitalCalibrator : public rclcpp::Node
{
public:
  VitalCalibrator()
  : Node("vital_calibrator")
  {
    // パラメータ
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 2000000);

    port_ = this->get_parameter("port").as_string();
    baud_ = this->get_parameter("baudrate").as_int();

    if (!openSerial()) {
      RCLCPP_FATAL(this->get_logger(), "Cannot open serial port %s", port_.c_str());
      rclcpp::shutdown();
      return;
    }

    // 「全基盤リセット」用トピック
    sub_all_ = this->create_subscription<Empty>(
      "reset_all_sensors", 10,
      std::bind(&VitalCalibrator::onResetAll, this, std::placeholders::_1));

    // 結果をまとめて返す
    pub_ack_ = this->create_publisher<String>("reset_sensor_ack", 10);

    RCLCPP_INFO(this->get_logger(),
      "Ready. Publish to 'reset_all_sensors' to reset ID:0x0A/0x0B/0x0C");
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

  void onResetAll(const Empty::SharedPtr /*msg*/)
  {
    const std::vector<uint8_t> ids = {0x0A, 0x0B, 0x0C};
    std::ostringstream summary;

    for (size_t i = 0; i < ids.size(); ++i) {
      uint8_t id = ids[i];
      bool ok = sendResetCommand(id);
      // "0x0A:OK" or "0x0B:FAIL" 形式で追記
      summary << "0x"
              << std::hex << std::uppercase
              << std::setw(2) << std::setfill('0')
              << static_cast<int>(id)
              << ":" << (ok ? "OK" : "FAIL");
      if (i + 1 < ids.size()) summary << "; ";
    }

    auto out = String();
    out.data = summary.str();
    pub_ack_->publish(out);

    RCLCPP_INFO(this->get_logger(), "Reset summary: %s", summary.str().c_str());
  }

  bool sendResetCommand(uint8_t id)
  {
    // コマンド送信 [AA C1 ID 22 55]
    uint8_t cmd[5] = {0xAA, 0xC1, id, 0x22, 0x55};
    if (write(fd_, cmd, 5) != 5) {
      return false;
    }

    // 応答受信 [AA C1 ID 00 55]
    uint8_t buf[5];
    ssize_t n = ::read(fd_, buf, 5);
    if (n != 5 ||
        buf[0] != 0xAA || buf[1] != 0xC1 ||
        buf[2] != id   || buf[3] != 0x00 ||
        buf[4] != 0x55)
    {
      return false;
    }
    return true;
  }

  std::string port_;
  int baud_;
  int fd_{-1};

  rclcpp::Subscription<Empty>::SharedPtr sub_all_;
  rclcpp::Publisher<String>::SharedPtr pub_ack_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VitalCalibrator>());
  rclcpp::shutdown();
  return 0;
}
