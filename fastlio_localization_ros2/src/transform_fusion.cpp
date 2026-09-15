#include <memory>
#include <mutex>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>

class TransformFusion : public rclcpp::Node
{
public:
    TransformFusion() : Node("transform_fusion")
    {
        T_map_to_odom_.setIdentity();
        T_odom_to_baselink_.setIdentity();

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        pub_localization_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization", 10);

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&TransformFusion::cb_odom_and_fusion, this, std::placeholders::_1));

        sub_map_to_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/map_to_odom", 10,
            std::bind(&TransformFusion::cb_save_map_to_odom, this, std::placeholders::_1));

        freq_pub_localization_ = 10;
        auto period_ms = static_cast<int64_t>(1000.0 / freq_pub_localization_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&TransformFusion::transform_fusion, this));

        RCLCPP_INFO(this->get_logger(), "TransformFusion node started");
    }

private:
    static Eigen::Matrix4d pose_to_mat(const geometry_msgs::msg::Pose& pose)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T(0, 3) = pose.position.x;
        T(1, 3) = pose.position.y;
        T(2, 3) = pose.position.z;
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z);
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        return T;
    }

    void transform_fusion()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!cur_odom_to_baselink_) return;

        Eigen::Matrix4d T_map_to_odom = T_map_to_odom_;

        // Broadcast map -> camera_init TF
        broadcast_map_to_camera_init(T_map_to_odom);

        // Compute and publish fused odometry (map -> body)
        publish_fused_odometry(T_map_to_odom);
    }

    void broadcast_map_to_camera_init(const Eigen::Matrix4d& T_map_to_odom)
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "camera_init";

        Eigen::Vector3d t = T_map_to_odom.block<3, 1>(0, 3);
        tf_msg.transform.translation.x = t.x();
        tf_msg.transform.translation.y = t.y();
        tf_msg.transform.translation.z = t.z();

        Eigen::Matrix3d R = T_map_to_odom.block<3, 3>(0, 0);
        Eigen::Quaterniond q(R);
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    void publish_fused_odometry(const Eigen::Matrix4d& T_map_to_odom)
    {
        Eigen::Matrix4d T_map_to_body = T_map_to_odom * T_odom_to_baselink_;

        nav_msgs::msg::Odometry localization;
        localization.header.stamp = this->now();
        localization.header.frame_id = "map";
        localization.child_frame_id = "body";

        Eigen::Vector3d t = T_map_to_body.block<3, 1>(0, 3);
        localization.pose.pose.position.x = t.x();
        localization.pose.pose.position.y = t.y();
        localization.pose.pose.position.z = t.z();

        Eigen::Matrix3d R = T_map_to_body.block<3, 3>(0, 0);
        Eigen::Quaterniond q(R);
        localization.pose.pose.orientation.x = q.x();
        localization.pose.pose.orientation.y = q.y();
        localization.pose.pose.orientation.z = q.z();
        localization.pose.pose.orientation.w = q.w();

        localization.twist = cur_odom_to_baselink_->twist;

        pub_localization_->publish(localization);
    }

    void cb_odom_and_fusion(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cur_odom_to_baselink_ = msg;
        T_odom_to_baselink_ = pose_to_mat(msg->pose.pose);
    }

    void cb_save_map_to_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        T_map_to_odom_ = pose_to_mat(msg->pose.pose);
    }

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_map_to_odom_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_localization_;

    // TF
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    nav_msgs::msg::Odometry::SharedPtr cur_odom_to_baselink_;
    Eigen::Matrix4d T_map_to_odom_;
    Eigen::Matrix4d T_odom_to_baselink_;
    double freq_pub_localization_;

    std::mutex mutex_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformFusion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
