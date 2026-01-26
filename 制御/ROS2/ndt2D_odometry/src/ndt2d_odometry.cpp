#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <cmath>

class NDT2DOdometry : public rclcpp::Node
{
public:
    NDT2DOdometry() : Node("ndt2d_odometry"), is_first_scan_(true)
    {
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("cell_size", 0.5);
        this->declare_parameter("max_iterations", 30);
        this->declare_parameter("convergence_threshold", 1e-4);
        this->declare_parameter("step_size", 0.1);
        this->declare_parameter("min_points_per_cell", 3);

        scan_topic_ = this->get_parameter("scan_topic").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        cell_size_ = this->get_parameter("cell_size").as_double();
        max_iterations_ = this->get_parameter("max_iterations").as_int();
        convergence_threshold_ = this->get_parameter("convergence_threshold").as_double();
        step_size_ = this->get_parameter("step_size").as_double();
        min_points_per_cell_ = this->get_parameter("min_points_per_cell").as_int();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&NDT2DOdometry::scanCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_ndt2d", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
    }

private:
    struct NDTCell
    {
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        Eigen::Matrix2d inv_cov = Eigen::Matrix2d::Zero();
        int count = 0;
        bool valid = false;
    };

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        std::vector<Eigen::Vector2d> points;
        points.reserve(scan->ranges.size());

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float r = scan->ranges[i];
            if (std::isfinite(r) && r >= scan->range_min && r <= scan->range_max) {
                float a = scan->angle_min + i * scan->angle_increment;
                points.emplace_back(r * std::cos(a), r * std::sin(a));
            }
        }

        if (points.size() < 50) {
            return;
        }

        if (is_first_scan_) {
            buildNDT(points);
            is_first_scan_ = false;
            publishOdometry(scan->header.stamp);
            return;
        }

        double dx = 0.0, dy = 0.0, dtheta = 0.0;
        alignNDT(points, dx, dy, dtheta);

        double c = std::cos(theta_);
        double s = std::sin(theta_);
        x_ += dx * c - dy * s;
        y_ += dx * s + dy * c;
        theta_ += dtheta;
        theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

        publishOdometry(scan->header.stamp);
        buildNDT(points);
    }

    int cellKey(double x, double y)
    {
        int ix = static_cast<int>(std::floor(x / cell_size_)) + 10000;
        int iy = static_cast<int>(std::floor(y / cell_size_)) + 10000;
        return iy * 100000 + ix;
    }

    void buildNDT(const std::vector<Eigen::Vector2d>& points)
    {
        ndt_grid_.clear();
        std::unordered_map<int, std::vector<Eigen::Vector2d>> buckets;

        for (const auto& p : points) {
            buckets[cellKey(p.x(), p.y())].push_back(p);
        }

        for (auto& [key, pts] : buckets) {
            if (static_cast<int>(pts.size()) < min_points_per_cell_) continue;

            NDTCell cell;
            cell.count = pts.size();

            for (const auto& p : pts) cell.mean += p;
            cell.mean /= pts.size();

            for (const auto& p : pts) {
                Eigen::Vector2d d = p - cell.mean;
                cell.cov += d * d.transpose();
            }
            cell.cov /= (pts.size() - 1);
            cell.cov += Eigen::Matrix2d::Identity() * 0.001;

            if (cell.cov.determinant() > 1e-6) {
                cell.inv_cov = cell.cov.inverse();
                cell.valid = true;
                ndt_grid_[key] = cell;
            }
        }
    }

    void alignNDT(const std::vector<Eigen::Vector2d>& points,
                  double& dx, double& dy, double& dtheta)
    {
        dx = dy = dtheta = 0.0;

        for (int iter = 0; iter < max_iterations_; ++iter) {
            double c = std::cos(dtheta);
            double s = std::sin(dtheta);

            Eigen::Vector3d g = Eigen::Vector3d::Zero();
            Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

            for (const auto& p : points) {
                double tx = c * p.x() - s * p.y() + dx;
                double ty = s * p.x() + c * p.y() + dy;

                auto it = ndt_grid_.find(cellKey(tx, ty));
                if (it == ndt_grid_.end() || !it->second.valid) continue;

                const NDTCell& cell = it->second;
                Eigen::Vector2d q(tx, ty);
                Eigen::Vector2d d = q - cell.mean;

                double e = d.transpose() * cell.inv_cov * d;
                if (e > 10.0) continue;

                double w = std::exp(-0.5 * e);
                Eigen::Vector2d Jt(-s * p.x() - c * p.y(), c * p.x() - s * p.y());

                Eigen::Matrix<double, 2, 3> J;
                J << 1, 0, Jt.x(),
                     0, 1, Jt.y();

                Eigen::Vector2d q_cov = cell.inv_cov * d;
                g += J.transpose() * q_cov * w;
                H += J.transpose() * cell.inv_cov * J * w;
            }

            H += Eigen::Matrix3d::Identity() * 1e-6;
            Eigen::Vector3d delta = H.ldlt().solve(-g);

            delta(0) = std::clamp(delta(0), -step_size_, step_size_);
            delta(1) = std::clamp(delta(1), -step_size_, step_size_);
            delta(2) = std::clamp(delta(2), -0.1, 0.1);

            dx += delta(0);
            dy += delta(1);
            dtheta += delta(2);

            if (delta.norm() < convergence_threshold_) break;
        }
    }

    void publishOdometry(const rclcpp::Time& stamp)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

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

        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = stamp;
            tf.header.frame_id = odom_frame_;
            tf.child_frame_id = base_frame_;
            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation = odom.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::unordered_map<int, NDTCell> ndt_grid_;
    double x_, y_, theta_;

    std::string scan_topic_, odom_frame_, base_frame_;
    bool publish_tf_;
    double cell_size_, convergence_threshold_, step_size_;
    int max_iterations_, min_points_per_cell_;
    bool is_first_scan_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NDT2DOdometry>());
    rclcpp::shutdown();
    return 0;
}
