#include <cstdint>
#include <chrono>
#include <memory>
#include <string>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robohachi_udp.hpp"

#define PACKET_SIZE 12
#define F7_PORT 4001

using namespace std::chrono;

class SendUDP : public rclcpp::Node
{
public:
  SendUDP(int f7_port) : Node("send_udp_auto"),
                         f7_addr((this->declare_parameter<std::string>("send_ip", "192.168.21.111"),
                                  this->get_parameter("send_ip").as_string())),
                         f7_udp(f7_addr, f7_port)
  {
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&SendUDP::cmd_vel_callback, this, std::placeholders::_1));

  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::string f7_addr;
  Ros2UDP f7_udp;
  uint8_t packet[PACKET_SIZE];

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Subscribing to /cmd_vel topic");
    float vel_x = static_cast<float>(msg->linear.x);
    float vel_y = static_cast<float>(msg->linear.y);
    float omega = static_cast<float>(msg->angular.z);

    std::memcpy(&packet[0], &vel_x, sizeof(float));
    std::memcpy(&packet[4], &vel_y, sizeof(float));
    std::memcpy(&packet[8], &omega, sizeof(float));

    f7_udp.send_packet(packet, PACKET_SIZE);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SendUDP>(F7_PORT);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
