#include <cstdio>
#include <chrono>
#include <string>
#include <memory>
#include <cstring>
#include <algorithm>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include <canable_msgs/msg/can.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono;

class CanRCV : public rclcpp::Node
{
public:
    CanRCV() : Node("can_rcv")
    {
        can_sub_ = this->create_subscription<canable_msgs::msg::Can>(
            "/can/receive", 10, std::bind(&CanRCV::can_Callback, this, std::placeholders::_1));
        
        wheel_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_wheel", 10);
        imu_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_imu", 10);
        
        odom_Imu_msg_.header.frame_id = "odom";
        odom_Imu_msg_.child_frame_id = "base_link";  
        
        odom_Wheel_msg_.header.frame_id = "odom";
        odom_Wheel_msg_.child_frame_id = "base_link";
        odom_Wheel_msg_.pose.pose.orientation.w = 1.0;
        flag_.fill(false);
    }
    
private:
    void can_Callback(const canable_msgs::msg::Can::SharedPtr can)
    {
        if(can->id == 0x101){
            odom_Wheel_msg_.pose.pose.position.y = -get_float(&can->data[0]);
            odom_Wheel_msg_.pose.pose.position.x = get_float(&can->data[4]);
            odom_Wheel_msg_.pose.pose.position.z = 0.0;
            flag_[0]=true;
        }
        else if(can->id == 0x102){ 
            odom_Wheel_msg_.twist.twist.linear.x = get_float(&can->data[0]);
            odom_Wheel_msg_.twist.twist.linear.y = get_float(&can->data[4]);
            flag_[1]=true;
        }
        else if(can->id == 0x151){
            odom_Imu_msg_.pose.pose.orientation.w = get_float(&can->data[0]);
            odom_Imu_msg_.pose.pose.orientation.x = get_float(&can->data[4]);
            flag_[2]=true;
        }
        else if(can->id == 0x152){
            odom_Imu_msg_.pose.pose.orientation.y = get_float(&can->data[0]);
            odom_Imu_msg_.pose.pose.orientation.z = get_float(&can->data[4]);
            flag_[3]=true;
        }
        else if(can->id == 0x153){
            odom_Imu_msg_.twist.twist.linear.x = get_float(&can->data[0]);
            odom_Imu_msg_.twist.twist.linear.y = get_float(&can->data[4]);
            flag_[4]=true;
        }
        else if(can->id == 0x154){
            odom_Imu_msg_.twist.twist.linear.z = get_float(&can->data[0]);
            odom_Imu_msg_.twist.twist.angular.x = get_float(&can->data[4]);
            flag_[5]=true;
        }
        else if(can->id == 0x155){
            odom_Imu_msg_.twist.twist.angular.y = get_float(&can->data[0]);
            odom_Imu_msg_.twist.twist.angular.z = get_float(&can->data[4]);
            flag_[6]=true; 
        }

        if(std::all_of(flag_.begin(), flag_.end(), [](bool b){ return b; })){
            odom_Wheel_msg_.header.stamp = this->now();
            odom_Imu_msg_.header.stamp = this->now();
            
            wheel_odom_pub_->publish(odom_Wheel_msg_);
            imu_odom_pub_->publish(odom_Imu_msg_);
            
            flag_.fill(false); 
        }
    }
    
    float get_float(const uint8_t* data) {
        return *reinterpret_cast<const float*>(data);
    }
    
    nav_msgs::msg::Odometry odom_Imu_msg_;
    nav_msgs::msg::Odometry odom_Wheel_msg_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_pub_;
    rclcpp::Subscription<canable_msgs::msg::Can>::SharedPtr can_sub_;
    
    std::array<bool, 7> flag_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanRCV>());
    rclcpp::shutdown();
    return 0;
}
