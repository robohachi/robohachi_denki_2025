#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector() : Node("aruco_detector")
    {
        this->declare_parameter("marker_size", 0.05);
        marker_size_ = this->get_parameter("marker_size").as_double();

        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        aruco_params_ = cv::aruco::DetectorParameters::create();
        aruco_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        aruco_params_->cornerRefinementWinSize = 5;
        aruco_params_->cornerRefinementMaxIterations = 30;
        aruco_params_->cornerRefinementMinAccuracy = 0.01;

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", 10,
            std::bind(&ArucoDetector::camera_info_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("aruco_pose", 10);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco_image", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; i++) {
            camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
        }
        dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); i++) {
            dist_coeffs_.at<double>(i) = msg->d[i];
        }
        camera_info_received_ = true;
    }

    // マーカーを太い線で描画
    void draw_marker(cv::Mat& image, const std::vector<cv::Point2f>& corners, int id, int thickness = 4)
    {
        // 四角形を描画
        std::vector<cv::Point> pts;
        for (const auto& c : corners) {
            pts.push_back(cv::Point(static_cast<int>(c.x), static_cast<int>(c.y)));
        }
        cv::polylines(image, pts, true, cv::Scalar(0, 255, 0), thickness);

        // ID表示
        cv::putText(image, "ID:" + std::to_string(id),
                    cv::Point(pts[0].x, pts[0].y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!camera_info_received_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            return;
        }
        cv::Mat image = cv_ptr->image;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, aruco_dict_, corners, ids, aruco_params_);

        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); i++) {
                // 太い線でマーカー描画
                draw_marker(image, corners[i], ids[i], 4);

                // 座標軸描画
                cv::drawFrameAxes(image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_size_ * 0.5);

                // 距離テキスト表示
                double distance = tvecs[i][2];
                std::string dist_text = "Z: " + std::to_string(distance).substr(0, 5) + "m";
                cv::Point2f center = (corners[i][0] + corners[i][2]) * 0.5f;
                cv::putText(image, dist_text, cv::Point(center.x, center.y - 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

                // クォータニオン変換
                cv::Mat rot_matrix;
                cv::Rodrigues(rvecs[i], rot_matrix);
                auto q = rotation_matrix_to_quaternion(rot_matrix);

                // ログ出力
                RCLCPP_INFO(this->get_logger(),
                    "ID:%d pos:[%.3f, %.3f, %.3f] quat:[%.3f, %.3f, %.3f, %.3f]",
                    ids[i],
                    tvecs[i][0], tvecs[i][1], tvecs[i][2],
                    q[0], q[1], q[2], q[3]);

                // PoseStamped publish
                auto pose_msg = geometry_msgs::msg::PoseStamped();
                pose_msg.header = msg->header;
                pose_msg.pose.position.x = tvecs[i][0];
                pose_msg.pose.position.y = tvecs[i][1];
                pose_msg.pose.position.z = tvecs[i][2];
                pose_msg.pose.orientation.x = q[0];
                pose_msg.pose.orientation.y = q[1];
                pose_msg.pose.orientation.z = q[2];
                pose_msg.pose.orientation.w = q[3];
                pose_pub_->publish(pose_msg);

                // TF broadcast
                geometry_msgs::msg::TransformStamped tf;
                tf.header = msg->header;
                tf.child_frame_id = "aruco_marker_" + std::to_string(ids[i]);
                tf.transform.translation.x = tvecs[i][0];
                tf.transform.translation.y = tvecs[i][1];
                tf.transform.translation.z = tvecs[i][2];
                tf.transform.rotation.x = q[0];
                tf.transform.rotation.y = q[1];
                tf.transform.rotation.z = q[2];
                tf.transform.rotation.w = q[3];
                tf_broadcaster_->sendTransform(tf);
            }
        }

        cv_ptr->image = image;
        image_pub_->publish(*cv_ptr->toImageMsg());
    }

    std::array<double, 4> rotation_matrix_to_quaternion(const cv::Mat& R)
    {
        double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
        double x, y, z, w;

        if (trace > 0) {
            double s = 0.5 / std::sqrt(trace + 1.0);
            w = 0.25 / s;
            x = (R.at<double>(2, 1) - R.at<double>(1, 2)) * s;
            y = (R.at<double>(0, 2) - R.at<double>(2, 0)) * s;
            z = (R.at<double>(1, 0) - R.at<double>(0, 1)) * s;
        } else if (R.at<double>(0, 0) > R.at<double>(1, 1) && R.at<double>(0, 0) > R.at<double>(2, 2)) {
            double s = 2.0 * std::sqrt(1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2));
            w = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
            x = 0.25 * s;
            y = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
            z = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
        } else if (R.at<double>(1, 1) > R.at<double>(2, 2)) {
            double s = 2.0 * std::sqrt(1.0 + R.at<double>(1, 1) - R.at<double>(0, 0) - R.at<double>(2, 2));
            w = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
            x = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
            y = 0.25 * s;
            z = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
        } else {
            double s = 2.0 * std::sqrt(1.0 + R.at<double>(2, 2) - R.at<double>(0, 0) - R.at<double>(1, 1));
            w = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
            x = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
            y = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
            z = 0.25 * s;
        }
        return {x, y, z, w};
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
    cv::Mat camera_matrix_, dist_coeffs_;
    double marker_size_;
    bool camera_info_received_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetector>());
    rclcpp::shutdown();
    return 0;
}
