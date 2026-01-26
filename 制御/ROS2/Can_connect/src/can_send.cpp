#include <cstdio>
#include <chrono>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <canable_msgs/msg/can.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono;

class CanSEND : public rclcpp::Node
{
public:
	CanSEND() : Node("can_send")
	{
		joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&CanSEND::joyCallback, this, std::placeholders::_1));
		can_pub_ = this->create_publisher<canable_msgs::msg::Can>("/can/transmit",10);

		timer_ = this->create_wall_timer(
            	std::chrono::milliseconds(300), [this]() {
                canable_msgs::msg::Can can_msg;
                can_msg.id = 0x000;
                can_msg.dlc = 1;
                memcpy(&can_msg.data, &canmsg.data, sizeof(canmsg.data));
                can_pub_->publish(can_msg);
            });

 		canmsg.data.emg = 0;
        canmsg.data.reset = 0;
        canmsg.x1 = 0.0;
        canmsg.y1 = 0.0;
        canmsg.x2 = 0.0;
        canmsg.y2 = 0.0;
        canmsg.data.reserved = 0;
	}
	
private:
	void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
    {
    	//RCLCPP_INFO(get_logger(),"%.2f  ,%.2f  ,%.2f  ,%.2f  ,%.2f  ,%.2f  ,%.2f  ,%.2f",joy->axes[0],joy->axes[1],joy->axes[2],joy->axes[3],joy->axes[4],joy->axes[5],joy->axes[6],joy->axes[7]);
    
		canmsg.x1 = joy->axes[0];
        canmsg.y1 = joy->axes[1];
        canmsg.x2 = joy->axes[3];
        canmsg.y2 = joy->axes[4];
        canmsg.data.reserved = 0;
        canable_msgs::msg::Can can_msg;

		can_msg.id = 0x000;
        can_msg.dlc = 1;
        memcpy(&can_msg.data, &canmsg.data, sizeof(canmsg.data));
        can_pub_->publish(can_msg);
        canmsg.data.reset = 0;

        can_msg.id = 0x001;
        can_msg.dlc = 8;
        memcpy(&can_msg.data[0], &canmsg.x1, sizeof(canmsg.x1));
        memcpy(&can_msg.data[4], &canmsg.y1, sizeof(canmsg.y1));
        
        can_pub_->publish(can_msg);
        canmsg.data.reset = 0;

        can_msg.id = 0x002;
        can_msg.dlc = 8;
        memcpy(&can_msg.data[0], &canmsg.x2, sizeof(canmsg.x2));
		memcpy(&can_msg.data[4], &canmsg.y2, sizeof(canmsg.y2));
        can_pub_->publish(can_msg);
        canmsg.data.reset = 0;
        RCLCPP_INFO(get_logger(),"%.2f,%.2f,%.2f,%.2f",canmsg.x1,canmsg.y1,canmsg.x2,canmsg.y2);
    }

	struct canmsg_s {
        struct {
            uint8_t emg : 1;
            uint8_t reset : 1;
            uint8_t reserved : 6;
        } data;
        float x1;
        float y1;
        float x2;
		float y2;
    } canmsg;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
	rclcpp::Publisher<canable_msgs::msg::Can>::SharedPtr can_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanSEND>());
    rclcpp::shutdown();
    return 0;
}
