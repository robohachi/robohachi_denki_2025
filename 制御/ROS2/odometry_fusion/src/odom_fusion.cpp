#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/Dense>
#include <mutex>
#include <cmath>

/**
 * カルマンフィルタ状態ベクトル:
 * state_(0): x位置 [m]
 * state_(1): y位置 [m]
 * state_(2): yaw角 [rad]
 * state_(3): x方向速度 vx [m/s]
 * state_(4): y方向速度 vy [m/s]
 * state_(5): 角速度 vyaw [rad/s]
 */

class OdomFusion : public rclcpp::Node
{
public:
    OdomFusion() : Node("odom_fusion")
    {
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("lidar_odom_topic", "/odom_ndt2d");
        this->declare_parameter<std::string>("fused_odom_topic", "/odom_fused");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<bool>("publish_tf", true);

        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string lidar_odom_topic = this->get_parameter("lidar_odom_topic").as_string();
        std::string fused_odom_topic = this->get_parameter("fused_odom_topic").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        state_ = Eigen::VectorXd::Zero(6);
        
        P_ = Eigen::MatrixXd::Identity(6, 6);
        P_(0, 0) = 0.01;
        P_(1, 1) = 0.01;
        P_(2, 2) = 0.01;
        P_(3, 3) = 0.1;
        P_(4, 4) = 0.1;
        P_(5, 5) = 0.05;
        
        Q_ = Eigen::MatrixXd::Identity(6, 6);
        Q_(0, 0) = 0.001;
        Q_(1, 1) = 0.001;
        Q_(2, 2) = 0.0001;
        Q_(3, 3) = 0.01;
        Q_(4, 4) = 0.01;
        Q_(5, 5) = 0.001;
        
        wheel_received_ = false;
        imu_received_ = false;
        lidar_received_ = false;
        
        wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&OdomFusion::wheel_odom_callback, this, std::placeholders::_1));
            
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&OdomFusion::imu_callback, this, std::placeholders::_1));
            
        lidar_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            lidar_odom_topic, 10,
            std::bind(&OdomFusion::lidar_odom_callback, this, std::placeholders::_1));
        
        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(fused_odom_topic, 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&OdomFusion::predict_and_publish, this));
            
        last_predict_time_ = this->now();
        initialized_ = false;
    }

private:
    void predict_and_publish()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!initialized_) return;
        
        auto current_time = this->now();
        double dt = (current_time - last_predict_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) {
            last_predict_time_ = current_time;
            return;
        }
        
        predict(dt);
        publish_odometry(current_time);
        
        last_predict_time_ = current_time;
    }
    
    void predict(double dt)
    {
        double yaw = state_(2);
        double vx = state_(3);
        double vy = state_(4);
        double vyaw = state_(5);
        
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        
        state_(0) += (vx * cos_yaw - vy * sin_yaw) * dt;
        state_(1) += (vx * sin_yaw + vy * cos_yaw) * dt;
        state_(2) += vyaw * dt;
        state_(2) = normalize_angle(state_(2));
        
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F(0, 2) = (-vx * sin_yaw - vy * cos_yaw) * dt;
        F(0, 3) = cos_yaw * dt;
        F(0, 4) = -sin_yaw * dt;
        F(1, 2) = (vx * cos_yaw - vy * sin_yaw) * dt;
        F(1, 3) = sin_yaw * dt;
        F(1, 4) = cos_yaw * dt;
        F(2, 5) = dt;
        
        P_ = F * P_ * F.transpose() + Q_;
        
        for (int i = 0; i < 6; i++) {
            P_(i, i) = std::clamp(P_(i, i), 1e-6, 100.0);
        }
    }
    
    void wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!wheel_received_) {
            wheel_received_ = true;
        }
        std::lock_guard<std::mutex> lock(mutex_);

        double v_mag = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        bool is_stopped = (v_mag < 0.01);

        if (is_stopped)
        {
            Eigen::VectorXd z(3);
            z(0) = msg->twist.twist.linear.x;
            z(1) = msg->twist.twist.linear.y;
            z(2) = 0.0;

            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
            H(0, 3) = 1;
            H(1, 4) = 1;
            H(2, 5) = 1;

            Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
            R(0, 0) = 0.001;
            R(1, 1) = 0.001;
            R(2, 2) = 0.0001;

            update(z, H, R);
        }
        else
        {
            Eigen::VectorXd z(2);
            z(0) = msg->twist.twist.linear.x;
            z(1) = msg->twist.twist.linear.y;

            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);
            H(0, 3) = 1;
            H(1, 4) = 1;

            Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
            R(0, 0) = 0.01;
            R(1, 1) = 0.01;

            update(z, H, R);
        }
        
        if (!initialized_) initialized_ = true;
    }
    
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!imu_received_) {
            imu_received_ = true;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!initialized_) {
            state_(2) = get_yaw_from_quaternion(msg->orientation);
            initialized_ = true;
            return;
        }
        
        Eigen::VectorXd z(2);
        z(0) = get_yaw_from_quaternion(msg->orientation);
        z(1) = msg->angular_velocity.z;
        
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);
        H(0, 2) = 1;
        H(1, 5) = 1;
        
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
        R(0, 0) = 0.01;
        R(1, 1) = 0.001;
        
        update_with_angle(z, H, R, 0);
    }
    
    void lidar_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!lidar_received_) {
            lidar_received_ = true;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!initialized_) {
            state_(0) = msg->pose.pose.position.x;
            state_(1) = msg->pose.pose.position.y;
            state_(2) = get_yaw_from_quaternion(msg->pose.pose.orientation);
            initialized_ = true;
            return;
        }
        
        Eigen::VectorXd z(3);
        z(0) = msg->pose.pose.position.x;
        z(1) = msg->pose.pose.position.y;
        z(2) = get_yaw_from_quaternion(msg->pose.pose.orientation);
        
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H(0, 0) = 1;
        H(1, 1) = 1;
        H(2, 2) = 1;
        
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
        R(0, 0) = 0.001;
        R(1, 1) = 0.001;
        R(2, 2) = 0.1;
        
        update_with_angle(z, H, R, 2);
    }
    
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R)
    {
        Eigen::MatrixXd S = H * P_ * H.transpose() + R;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        Eigen::VectorXd y = z - H * state_;
        
        state_ = state_ + K * y;
        
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd IKH = I - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
        
        state_(2) = normalize_angle(state_(2));
    }
    
    void update_with_angle(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R, int angle_idx)
    {
        Eigen::MatrixXd S = H * P_ * H.transpose() + R;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        Eigen::VectorXd y = z - H * state_;
        
        y(angle_idx) = normalize_angle(y(angle_idx));
        
        state_ = state_ + K * y;
        
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd IKH = I - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
        
        state_(2) = normalize_angle(state_(2));
    }
    
    void publish_odometry(const rclcpp::Time& stamp)
    {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;
        
        odom_msg.pose.pose.position.x = state_(0);
        odom_msg.pose.pose.position.y = state_(1);
        odom_msg.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, state_(2));
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        
        odom_msg.twist.twist.linear.x = state_(3);
        odom_msg.twist.twist.linear.y = state_(4);
        odom_msg.twist.twist.angular.z = state_(5);
        
        for (int i = 0; i < 36; i++) {
            odom_msg.pose.covariance[i] = 0.0;
            odom_msg.twist.covariance[i] = 0.0;
        }
        odom_msg.pose.covariance[0] = P_(0, 0);
        odom_msg.pose.covariance[7] = P_(1, 1);
        odom_msg.pose.covariance[35] = P_(2, 2);
        odom_msg.twist.covariance[0] = P_(3, 3);
        odom_msg.twist.covariance[7] = P_(4, 4);
        odom_msg.twist.covariance[35] = P_(5, 5);
        
        fused_odom_pub_->publish(odom_msg);
        
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = stamp;
            transform.header.frame_id = odom_frame_;
            transform.child_frame_id = base_frame_;
            transform.transform.translation.x = state_(0);
            transform.transform.translation.y = state_(1);
            transform.transform.translation.z = 0.0;
            transform.transform.rotation = odom_msg.pose.pose.orientation;
            
            tf_broadcaster_->sendTransform(transform);
        }
    }
    
    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    Eigen::VectorXd state_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    
    bool wheel_received_;
    bool imu_received_;
    bool lidar_received_;
    
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Time last_predict_time_;
    bool initialized_;
    std::mutex mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomFusion>());
    rclcpp::shutdown();
    return 0;
}
