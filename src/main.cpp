#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <iomanip>
#include <sstream>

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

    // 起動と同時にキャリブレーション実行
    std::string summary = calibrateAll();
    RCLCPP_INFO(this->get_logger(), "Calibration result: %s", summary.c_str());

    // ノードを終了
    rclcpp::shutdown();
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
    for (size_t i = 0; i < ids.size(); ++i) {
      uint8_t id = ids[i];
      bool ok = sendResetCommand(id);
      summary << "0x"
              << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
              << static_cast<int>(id)
              << ":" << (ok ? "OK" : "FAIL");
      if (i + 1 < ids.size()) summary << "; ";
    }
    return summary.str();
  }

  bool sendResetCommand(uint8_t id)
  {
    uint8_t cmd[5] = {0xAA, 0xC1, id, 0x22, 0x55};
    if (write(fd_, cmd, sizeof(cmd)) != sizeof(cmd)) return false;

    uint8_t buf[5];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n != sizeof(buf) ||
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
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // コンストラクタ内でキャリブレーションと終了を行う
  std::make_shared<VitalCalibrator>();
  return 0;
}
