// encoder_odom.cpp
// ROS 2 Humble - Differential drive odometry from encoder ticks over Serial
//
// Publishes:
//   - nav_msgs/Odometry on /odom
//   - TF transform odom -> base_link
//
// Key design:
//   - Only "update ticks" when new serial data arrives
//   - BUT publish odom+tf at a fixed rate regardless (so /odom never "stalls")
//
// Parameters:
//   port (string)               default: /dev/ttyUSB0
//   baud (int)                  default: 115200
//   wheel_radius (double, m)    default: 0.02
//   wheel_separation (double,m) default: 0.16
//   encoder_cpr (int)           default: 4096     // counts per wheel revolution (after x4 if quadrature)
//   publish_rate (double, Hz)   default: 30.0
//   frame_id (string)           default: odom
//   child_frame_id (string)     default: base_link

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <cmath>
#include <string>
#include <sstream>
#include <optional>
#include <vector>
#include <regex>

class SerialPort {
public:
  SerialPort() = default;
  ~SerialPort() { closePort(); }

  bool openPort(const std::string& port, int baud) {
    closePort();
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      closePort();
      return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;      // 1 stop bit
    tty.c_cflag &= ~PARENB;      // no parity
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;          // 8 data bits
    tty.c_cflag &= ~CRTSCTS;     // no HW flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // no SW flow control

    speed_t sp = baudToSpeed(baud);
    if (sp == 0) {
      closePort();
      return false;
    }
    cfsetispeed(&tty, sp);
    cfsetospeed(&tty, sp);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      closePort();
      return false;
    }

    port_ = port;
    baud_ = baud;
    return true;
  }

  void closePort() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    buffer_.clear();
  }

  bool isOpen() const { return fd_ >= 0; }
  std::string port() const { return port_; }
  int baud() const { return baud_; }

  // Non-blocking: read available bytes, return complete lines (without \r\n)
  std::vector<std::string> readLines() {
    std::vector<std::string> lines;
    if (fd_ < 0) return lines;

    // Check if bytes available
    int bytes_available = 0;
    ioctl(fd_, FIONREAD, &bytes_available);
    if (bytes_available <= 0) return lines;

    std::string tmp;
    tmp.resize(static_cast<size_t>(bytes_available));
    int n = ::read(fd_, tmp.data(), tmp.size());
    if (n <= 0) return lines;

    buffer_.append(tmp.data(), static_cast<size_t>(n));

    // Split by newline
    size_t pos = 0;
    while (true) {
      size_t nl = buffer_.find('\n', pos);
      if (nl == std::string::npos) break;
      std::string line = buffer_.substr(0, nl);
      buffer_.erase(0, nl + 1);

      // Trim CR
      if (!line.empty() && line.back() == '\r') line.pop_back();

      // Trim spaces
      line = trim(line);
      if (!line.empty()) lines.push_back(line);

      pos = 0;
    }
    // Prevent runaway buffer if no newline
    if (buffer_.size() > 4096) buffer_.clear();

    return lines;
  }

private:
  static speed_t baudToSpeed(int baud) {
    switch (baud) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      default: return 0;
    }
  }

  static std::string trim(const std::string& s) {
    const char* ws = " \t\n\r";
    size_t b = s.find_first_not_of(ws);
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
  }

  int fd_{-1};
  std::string port_{};
  int baud_{0};
  std::string buffer_{};
};

struct EncTicks {
  int64_t left{0};
  int64_t right{0};
};

static std::optional<EncTicks> parseEncoderLine(const std::string& line) {
  // Accept:
  // 1) "E L=123 R=456"
  // 2) "L=123 R=456"
  // 3) "L:123 R:456"
  // 4) "123,456"
  // 5) "L 123 R 456"
  //
  // We'll try multiple regex patterns.
  static const std::regex re_lr_eq(R"((?:^|.*\s)L\s*=?\s*([-+]?\d+)\s+R\s*=?\s*([-+]?\d+).*$)", std::regex::icase);
  static const std::regex re_lr_colon(R"((?:^|.*\s)L\s*:\s*([-+]?\d+)\s+R\s*:\s*([-+]?\d+).*$)", std::regex::icase);
  static const std::regex re_csv(R"(^\s*([-+]?\d+)\s*,\s*([-+]?\d+)\s*$)");

  std::smatch m;
  if (std::regex_match(line, m, re_csv)) {
    EncTicks t;
    t.left = std::stoll(m[1]);
    t.right = std::stoll(m[2]);
    return t;
  }
  if (std::regex_match(line, m, re_lr_eq)) {
    EncTicks t;
    t.left = std::stoll(m[1]);
    t.right = std::stoll(m[2]);
    return t;
  }
  if (std::regex_match(line, m, re_lr_colon)) {
    EncTicks t;
    t.left = std::stoll(m[1]);
    t.right = std::stoll(m[2]);
    return t;
  }

  // If line begins with "E " often: try removing leading token and retry eq pattern
  if (line.size() > 2 && (line[0] == 'E' || line[0] == 'e')) {
    std::string rest = line.substr(1);
    if (std::regex_match(rest, m, re_lr_eq)) {
      EncTicks t;
      t.left = std::stoll(m[1]);
      t.right = std::stoll(m[2]);
      return t;
    }
    if (std::regex_match(rest, m, re_lr_colon)) {
      EncTicks t;
      t.left = std::stoll(m[1]);
      t.right = std::stoll(m[2]);
      return t;
    }
  }

  return std::nullopt;
}

class EncoderOdomNode : public rclcpp::Node {
public:
  EncoderOdomNode()
  : Node("encoder_odom"),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    // Params
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_ = this->declare_parameter<int>("baud", 115200);
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.02);
    wheel_separation_ = this->declare_parameter<double>("wheel_separation", 0.16);
    encoder_cpr_ = this->declare_parameter<int>("encoder_cpr", 4096);
    publish_rate_ = this->declare_parameter<double>("publish_rate", 30.0);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "odom");
    child_frame_id_ = this->declare_parameter<std::string>("child_frame_id", "base_link");

    if (encoder_cpr_ <= 0) {
      RCLCPP_WARN(get_logger(), "encoder_cpr <= 0, forcing to 1");
      encoder_cpr_ = 1;
    }
    if (publish_rate_ <= 0.0) publish_rate_ = 30.0;

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Open serial
    if (!serial_.openPort(port_, baud_)) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial %s @ %d", port_.c_str(), baud_);
    } else {
      RCLCPP_INFO(get_logger(), "Serial open: %s @ %d", serial_.port().c_str(), serial_.baud());
    }

    last_time_ = this->now();
    last_publish_time_ = last_time_;
    have_ticks_ = false;

    // Timer publish loop
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&EncoderOdomNode::onTimer, this)
    );
  }

private:
  void onTimer() {
    // 1) Try read new serial lines; update latest ticks if found
    bool got_new_ticks = false;
    if (serial_.isOpen()) {
      auto lines = serial_.readLines();
      for (const auto& line : lines) {
        auto parsed = parseEncoderLine(line);
        if (parsed.has_value()) {
          latest_ticks_ = parsed.value();
          got_new_ticks = true;
          have_ticks_ = true;
          // keep last valid line only; continue scanning lines (ok)
        }
      }
    }

    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 1e-6) dt = 1e-6;

    // 2) Integrate odom using delta ticks when available
    // If no ticks ever received -> publish zeros but still publish.
    int64_t dl = 0;
    int64_t dr = 0;

    if (have_ticks_) {
      if (!have_prev_ticks_) {
        prev_ticks_ = latest_ticks_;
        have_prev_ticks_ = true;
      }

      if (got_new_ticks) {
        dl = latest_ticks_.left  - prev_ticks_.left;
        dr = latest_ticks_.right - prev_ticks_.right;
        prev_ticks_ = latest_ticks_;
      } else {
        dl = 0;
        dr = 0;
      }
    }

    // Convert tick delta to distance
    const double meters_per_tick = (2.0 * M_PI * wheel_radius_) / static_cast<double>(encoder_cpr_);
    const double dL = static_cast<double>(dl) * meters_per_tick;
    const double dR = static_cast<double>(dr) * meters_per_tick;

    const double dS = (dR + dL) * 0.5;
    const double dTheta = (dR - dL) / wheel_separation_;

    // Midpoint integration
    const double theta_mid = theta_ + dTheta * 0.5;
    x_ += dS * std::cos(theta_mid);
    y_ += dS * std::sin(theta_mid);
    theta_ = normalizeAngle(theta_ + dTheta);

    // Velocities (if no new ticks -> 0)
    const double vx = dS / dt;
    const double vth = dTheta / dt;

    // 3) Publish odom + TF ALWAYS (even if got_new_ticks == false)
    publishOdomAndTF(now, vx, vth);

    last_time_ = now;
  }

  void publishOdomAndTF(const rclcpp::Time& stamp, double vx, double vth) {
    // quaternion from yaw
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    q.normalize();

    // TF: odom -> base_link
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = frame_id_;
    tf_msg.child_frame_id = child_frame_id_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);

    // Odometry msg
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;

    // Simple covariance (tune later)
    // If you want: larger covariance when not moving / no encoder, etc.
    for (int i = 0; i < 36; i++) {
      odom.pose.covariance[i] = 0.0;
      odom.twist.covariance[i] = 0.0;
    }
    odom.pose.covariance[0] = 0.02;   // x
    odom.pose.covariance[7] = 0.02;   // y
    odom.pose.covariance[35] = 0.05;  // yaw
    odom.twist.covariance[0] = 0.05;  // vx
    odom.twist.covariance[35] = 0.1;  // wz

    odom_pub_->publish(odom);
  }

  static double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // Params
  std::string port_;
  int baud_;
  double wheel_radius_;
  double wheel_separation_;
  int encoder_cpr_;
  double publish_rate_;
  std::string frame_id_;
  std::string child_frame_id_;

  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Serial
  SerialPort serial_;

  // State
  bool have_ticks_{false};
  bool have_prev_ticks_{false};
  EncTicks latest_ticks_{};
  EncTicks prev_ticks_{};

  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};

  rclcpp::Time last_time_;
  rclcpp::Time last_publish_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EncoderOdomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

