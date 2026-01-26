#include <cstdio>
#include <chrono>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <canable_msgs/msg/can.hpp>
#include <geometry_msgs/msg/twist.hpp>

class CanSEND : public rclcpp::Node
{
public:
    CanSEND() : Node("can_send_auto")
    {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CanSEND::cmdVelCallback, this, std::placeholders::_1));
        can_pub_ = this->create_publisher<canable_msgs::msg::Can>("/can/transmit", 10);

        // ハートビート用タイマー（ID 0x000のみ）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(300), [this]() {
                canable_msgs::msg::Can can_msg;
                can_msg.id = 0x000;
                can_msg.dlc = 1;
                memcpy(&can_msg.data, &canmsg_.data, sizeof(canmsg_.data));
                can_pub_->publish(can_msg);
            });

        canmsg_.data.emg = 0;
        canmsg_.data.reset = 0;
        canmsg_.data.reserved = 0;
        canmsg_.x1 = 0.0f;
        canmsg_.y1 = 0.0f;
        canmsg_.x2 = 0.0f;
        canmsg_.y2 = 0.0f;
    }
    
private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
    {
        canmsg_.x1 = static_cast<float>(cmd->linear.y);
        canmsg_.y1 = static_cast<float>(cmd->linear.x);
        canmsg_.x2 = static_cast<float>(cmd->angular.z);
        canmsg_.y2 = 0.0f;

        canable_msgs::msg::Can can_msg;

        can_msg.id = 0x001;
        can_msg.dlc = 8;
        memcpy(&can_msg.data[0], &canmsg_.x1, sizeof(canmsg_.x1));
        memcpy(&can_msg.data[4], &canmsg_.y1, sizeof(canmsg_.y1));
        can_pub_->publish(can_msg);

        can_msg.id = 0x002;
        can_msg.dlc = 8;
        memcpy(&can_msg.data[0], &canmsg_.x2, sizeof(canmsg_.x2));
        memcpy(&can_msg.data[4], &canmsg_.y2, sizeof(canmsg_.y2));
        can_pub_->publish(can_msg);
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
    } canmsg_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
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
