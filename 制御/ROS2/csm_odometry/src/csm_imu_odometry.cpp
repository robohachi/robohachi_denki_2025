#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class ScanMatchingIMUOdometry : public rclcpp::Node
{
public:
    ScanMatchingIMUOdometry() : Node("csm_imu_odometry"), is_first_scan_(true), has_imu_(false)
    {
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("imu_topic", "/imu");
        this->declare_parameter("use_imu", true);
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_topic").as_string(), 10,
            std::bind(&ScanMatchingIMUOdometry::scanCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic").as_string(), 10,
            std::bind(&ScanMatchingIMUOdometry::imuCallback, this, std::placeholders::_1));
        
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_csm", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        use_imu_ = this->get_parameter("use_imu").as_bool();
        x_ = y_ = theta_ = 0.0;
        imu_theta_ = prev_imu_theta_ = 0.0;
        last_imu_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Scan Matching + IMU Odometry node initialized");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
    {
        if (!use_imu_) return;

        tf2::Quaternion q(imu->orientation.x, imu->orientation.y,
                         imu->orientation.z, imu->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        if (!has_imu_) {
            imu_theta_ = yaw;
            has_imu_ = true;
        } else {
            rclcpp::Time current_time = imu->header.stamp;
            double dt = (current_time - last_imu_time_).seconds();
            if (dt > 0.0 && dt < 1.0) {
                imu_theta_ += imu->angular_velocity.z * dt;
            }
        }
        last_imu_time_ = imu->header.stamp;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = laserScanToPointCloud(scan);
        
        if (cloud->points.size() < 10) {
            RCLCPP_WARN(this->get_logger(), "Scan points too few: %zu", cloud->points.size());
            return;
        }
        
        if (is_first_scan_) {
            prev_cloud_ = cloud;
            is_first_scan_ = false;
            return;
        }

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud);
        icp.setInputTarget(prev_cloud_);
        icp.setMaxCorrespondenceDistance(0.5);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);

        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);

        if (icp.hasConverged()) {
            Eigen::Matrix4f transformation = icp.getFinalTransformation();
            
            double dx = transformation(0, 3);
            double dy = transformation(1, 3);
            double dtheta = std::atan2(transformation(1, 0), transformation(0, 0));

            x_ += dx * std::cos(theta_) - dy * std::sin(theta_);
            y_ += dx * std::sin(theta_) + dy * std::cos(theta_);
            theta_ += dtheta;

            publishOdometry(scan->header.stamp);
            prev_cloud_ = cloud;
            prev_imu_theta_ = imu_theta_;
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP matching failed");
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanToPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            if (scan->ranges[i] >= scan->range_min && scan->ranges[i] <= scan->range_max) {
                double angle = scan->angle_min + i * scan->angle_increment;
                pcl::PointXYZ point;
                point.x = scan->ranges[i] * std::cos(angle);
                point.y = scan->ranges[i] * std::sin(angle);
                point.z = 0.0;
                cloud->points.push_back(point);
            }
        }
        
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;
        
        return cloud;
    }

    void publishOdometry(const rclcpp::Time& stamp)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = odom.pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
    
    bool is_first_scan_, has_imu_, use_imu_;
    double x_, y_, theta_;
    double imu_theta_, prev_imu_theta_;
    rclcpp::Time last_imu_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMatchingIMUOdometry>());
    rclcpp::shutdown();
    return 0;
}
