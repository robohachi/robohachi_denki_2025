#include "robohachi_udp.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <sys/socket.h>
#include <string>
#include <cstring>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <math.h>
#include <algorithm>
#include <chrono>
#include <thread>

#define BUFFER_SIZE 96
#define ANY_ADDR "0.0.0.0"
#define F7_PORT 4001
#define LOOP_INTERVAL_US 1000

class ReceiveUDP : public rclcpp::Node{
    public:
        ReceiveUDP(const std::string& f7_address, int f7_port)
            :Node("receive_udp"), f7_udp(f7_address, f7_port){
                f7_udp.udp_bind();
                
                rclcpp::QoS qos(rclcpp::KeepLast(10));
                pub_imu = create_publisher<sensor_msgs::msg::Imu>("/imu_data", qos);
                pub_odom = create_publisher<nav_msgs::msg::Odometry>("/odom_wheel", qos);
                tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            }
            
        void receive_data(){
            uint8_t buf[BUFFER_SIZE] = {0};
            ssize_t received_length;
            
            uint8_t latest_buf[BUFFER_SIZE];
            int packet_count = 0;
            while((received_length = f7_udp.udp_recv(buf, sizeof(buf))) > 0) {
                if(received_length >= 96) {
                    memcpy(latest_buf, buf, BUFFER_SIZE);
                    packet_count++;
                }
            }
            
            if(packet_count > 0) {
                process_imu_data(latest_buf);
                process_odom_data(latest_buf);
            }
        }
        
    private:
        void process_imu_data(uint8_t* buf) {
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = "imu_link";
            
            float accel_x = *(float*)(&buf[4]);
            float accel_y = *(float*)(&buf[8]);
            float accel_z = *(float*)(&buf[12]);
            float gyro_x = *(float*)(&buf[16]);
            float gyro_y = *(float*)(&buf[20]);
            float gyro_z = *(float*)(&buf[24]);
            
            imu_msg.linear_acceleration.x = accel_x;
            imu_msg.linear_acceleration.y = accel_y;
            imu_msg.linear_acceleration.z = accel_z;
            
            imu_msg.angular_velocity.x = gyro_x;
            imu_msg.angular_velocity.y = gyro_y;
            imu_msg.angular_velocity.z = gyro_z;
            
            float quat_w = *(float*)(&buf[80]);
            float quat_x = *(float*)(&buf[84]);
            float quat_y = *(float*)(&buf[88]);
            float quat_z = *(float*)(&buf[92]);
            
            imu_msg.orientation.w = quat_w;
            imu_msg.orientation.x = quat_x;
            imu_msg.orientation.y = quat_y;
            imu_msg.orientation.z = quat_z;
            
            for (int i = 0; i < 9; i++) {
                imu_msg.orientation_covariance[i] = 0.0;
                imu_msg.angular_velocity_covariance[i] = 0.0;
                imu_msg.linear_acceleration_covariance[i] = 0.0;
            }
            imu_msg.orientation_covariance[0] = 0.01;
            imu_msg.orientation_covariance[4] = 0.01;
            imu_msg.orientation_covariance[8] = 0.01;
            imu_msg.angular_velocity_covariance[0] = 0.01;
            imu_msg.angular_velocity_covariance[4] = 0.01;
            imu_msg.angular_velocity_covariance[8] = 0.01;
            imu_msg.linear_acceleration_covariance[0] = 0.01;
            imu_msg.linear_acceleration_covariance[4] = 0.01;
            imu_msg.linear_acceleration_covariance[8] = 0.01;
            
            pub_imu->publish(imu_msg);
        }

        void process_odom_data(uint8_t* buf) {
            auto odom_msg = nav_msgs::msg::Odometry();
            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "wheel_odom";
            odom_msg.child_frame_id = "base_link";
    
            float mcu_x = *(float*)(&buf[52]);
            float mcu_y = *(float*)(&buf[56]);
            float mcu_yaw = *(float*)(&buf[60]);
            float mcu_vx = *(float*)(&buf[64]);
            float mcu_vy = *(float*)(&buf[68]);
            float mcu_omega = *(float*)(&buf[72]);
    
            odom_msg.pose.pose.position.x = mcu_y;
            odom_msg.pose.pose.position.y = mcu_x;
            odom_msg.pose.pose.position.z = 0.0;
    
            tf2::Quaternion q;
            q.setRPY(0, 0, mcu_yaw);
            odom_msg.pose.pose.orientation.w = q.w();
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();

            odom_msg.twist.twist.linear.x = mcu_vy;
            odom_msg.twist.twist.linear.y = -mcu_vx;
            odom_msg.twist.twist.linear.z = 0.0;
            odom_msg.twist.twist.angular.x = 0.0;
            odom_msg.twist.twist.angular.y = 0.0;
            odom_msg.twist.twist.angular.z = -mcu_omega;
    

    
            pub_odom->publish(odom_msg);
    
            geometry_msgs::msg::TransformStamped transform;
            transform.header = odom_msg.header;
            transform.child_frame_id = "base_link";
            transform.transform.translation.x = odom_msg.pose.pose.position.x;
            transform.transform.translation.y = odom_msg.pose.pose.position.y;
            transform.transform.translation.z = 0.0;
            transform.transform.rotation = odom_msg.pose.pose.orientation;
    
            tf_broadcaster_->sendTransform(transform);
        }
        
        Ros2UDP f7_udp;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ReceiveUDP>(ANY_ADDR, F7_PORT);
    
    while (rclcpp::ok()) {
        auto start = std::chrono::high_resolution_clock::now();
        
        node->receive_data();
        rclcpp::spin_some(node);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        if (elapsed.count() < LOOP_INTERVAL_US) {
           std::this_thread::sleep_for(std::chrono::microseconds(LOOP_INTERVAL_US - elapsed.count()));
        }
    }
    
    rclcpp::shutdown();
    return 0;
}
