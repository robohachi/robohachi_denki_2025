#include <cstdint>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "robohachi_udp.hpp"
#include "stdint.h"

#define PACKET_SIZE 21 
#define F7_PORT 4001

using namespace std::chrono;

class SendUDP : public rclcpp::Node
{
  public : SendUDP(int f7_port) : Node("send_udp"),
                        f7_addr((this->declare_parameter<std::string>("send_ip", "192.168.21.111"),
                                 this->get_parameter("send_ip").as_string())),
                        f7_udp(f7_addr, f7_port){
    
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy",10,std::bind(&SendUDP::joy_callback, this, std::placeholders::_1));
    
    //self.stop_service = self.create_service(Stop,'move_stop',self.move_stop_callback)
    
    RCLCPP_INFO(this->get_logger(), "Subscribing to /joy topic");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  std::string f7_addr;
  Ros2UDP f7_udp;
  std::vector<uint8_t> packet;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    packet.resize(PACKET_SIZE);

    // アナログスティックの値を unsigned char に変換
    for (size_t i= 0; i < msg->axes.size(); ++i) {
      uint8_t scaled_value = static_cast<uint8_t>((msg->axes[i] + 1.0) / 2.0 * 255.0);
      if (i < packet.size()) {
        packet[i] = scaled_value;
    }
  
      // 例: float の値を 0-255 の範囲にスケーリング (必要に応じて調整)  }

    // ボタンの状態を格納
    for (size_t i = 0; i < msg->buttons.size(); ++i) {
      if (msg->axes.size() + i < packet.size()) {
        packet[msg->axes.size() + i] = static_cast<uint8_t>(msg->buttons[i]);
      }
    }

    // 送信処理(robohachi_udp.hpp で定義されている Ros2UDP クラスの send_packet 関数を使用)
    f7_udp.send_packet(packet.data(), static_cast<uint8_t>(packet.size()));

    // 送信したデータを表示
    //RCLCPP_INFO(this->get_logger(),"send: %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u",packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7],packet[8], packet[9], packet[10], packet[11], packet[12], packet[13], packet[14], packet[15],packet[16], packet[17], packet[18], packet[19], packet[20]);
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SendUDP>(F7_PORT);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
