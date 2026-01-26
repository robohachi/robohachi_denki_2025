#include "rclcpp/rclcpp.hpp"
#include "yolo_msgs/msg/detection_array.hpp"
#include "yolo_position_msgs/msg/yolo_position.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pid_controller.hpp"
#include <algorithm>
#include <vector>
#include <cmath>

class ObjectFollower : public rclcpp::Node
{
public:
  ObjectFollower() : Node("yolo_position")
  {
    declare_parameter("image_width", 640.0);
    declare_parameter("target_size", 100.0);
    declare_parameter("target_classes", std::vector<std::string>{"bottle"});
    declare_parameter("angular_kp", 1.0);
    declare_parameter("angular_ki", 0.0);
    declare_parameter("angular_kd", 0.1);
    declare_parameter("linear_kp", 0.005);
    declare_parameter("linear_ki", 0.0);
    declare_parameter("linear_kd", 0.001);
    declare_parameter("detection_timeout", 0.5);
    declare_parameter("angular_deadband", 0.05);
    declare_parameter("linear_deadband", 0.02);
    declare_parameter("smoothing_factor", 0.3);

    image_width_ = get_parameter("image_width").as_double();
    target_size_ = get_parameter("target_size").as_double();
    target_classes_ = get_parameter("target_classes").as_string_array();
    detection_timeout_ = get_parameter("detection_timeout").as_double();
    angular_deadband_ = get_parameter("angular_deadband").as_double();
    linear_deadband_ = get_parameter("linear_deadband").as_double();
    smoothing_factor_ = get_parameter("smoothing_factor").as_double();

    angular_pid_ = PIDController(
      get_parameter("angular_kp").as_double(),
      get_parameter("angular_ki").as_double(),
      get_parameter("angular_kd").as_double(),
      -1.0, 1.0);

    linear_pid_ = PIDController(
      get_parameter("linear_kp").as_double(),
      get_parameter("linear_ki").as_double(),
      get_parameter("linear_kd").as_double(),
      -1.0, 1.0);

    subscription_ = create_subscription<yolo_msgs::msg::DetectionArray>(
      "/yolo/detections", 10,
      std::bind(&ObjectFollower::detection_callback, this, std::placeholders::_1));

    position_pub_ = create_publisher<yolo_position_msgs::msg::YoloPosition>("/yolo_position", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    last_time_ = this->now();
    last_detection_time_ = this->now();
    prev_linear_x_ = 0.0;
    prev_angular_z_ = 0.0;
  }

private:
  double apply_deadband(double value, double deadband)
  {
    return std::abs(value) < deadband ? 0.0 : value;
  }

  double smooth_velocity(double current, double previous, double factor)
  {
    return previous + factor * (current - previous);
  }

  void detection_callback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
  {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    auto position = yolo_position_msgs::msg::YoloPosition();
    position.header = msg->header;

    auto cmd = geometry_msgs::msg::Twist();
    bool detected = false;

    for (const auto& detection : msg->detections)
    {
      const std::string& class_name = detection.class_name;

      bool is_target = std::find(target_classes_.begin(), target_classes_.end(), class_name) 
                       != target_classes_.end();

      if (is_target)
      {
        position.class_name = class_name;
        position.x = detection.bbox.center.position.x;
        position.y = detection.bbox.center.position.y;
        position.width = detection.bbox.size.x;
        position.height = detection.bbox.size.y;
        position.score = detection.score;

        double normalized_x = (position.x - image_width_ / 2.0) / (image_width_ / 2.0);
        double angular_error = -normalized_x;
        double angular_raw = angular_pid_.calculate(angular_error, dt);
        double angular_with_deadband = apply_deadband(angular_raw, angular_deadband_);
        cmd.angular.z = smooth_velocity(angular_with_deadband, prev_angular_z_, smoothing_factor_);

        double current_size = std::max(position.width, position.height);
        double size_error = target_size_ - current_size;
        double linear_raw = linear_pid_.calculate(size_error, dt);
        double linear_with_deadband = apply_deadband(linear_raw, linear_deadband_);
        cmd.linear.x = smooth_velocity(linear_with_deadband, prev_linear_x_, smoothing_factor_);

        prev_linear_x_ = cmd.linear.x;
        prev_angular_z_ = cmd.angular.z;
        last_detection_time_ = now;

        detected = true;
        break;
      }
    }

    if (!detected)
    {
      double time_since_detection = (now - last_detection_time_).seconds();
      if (time_since_detection > detection_timeout_)
      {
        angular_pid_.reset();
        linear_pid_.reset();
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        prev_linear_x_ = 0.0;
        prev_angular_z_ = 0.0;
      }
      else
      {
        cmd.linear.x = prev_linear_x_;
        cmd.angular.z = prev_angular_z_;
      }
    }


    RCLCPP_INFO(get_logger(), "x=%.1f, y=%.1f, linear.x=%.3f, rotate=%.3f",
          position.x, position.y,cmd.linear.x, cmd.angular.z);

    position_pub_->publish(position);
    cmd_vel_pub_->publish(cmd);
  }

  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr subscription_;
  rclcpp::Publisher<yolo_position_msgs::msg::YoloPosition>::SharedPtr position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  PIDController angular_pid_;
  PIDController linear_pid_;
  double image_width_;
  double target_size_;
  std::vector<std::string> target_classes_;
  double detection_timeout_;
  double angular_deadband_;
  double linear_deadband_;
  double smoothing_factor_;
  rclcpp::Time last_time_;
  rclcpp::Time last_detection_time_;
  double prev_linear_x_;
  double prev_angular_z_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectFollower>());
  rclcpp::shutdown();
  return 0;
}
