#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <mutex>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloud::Ptr;

class GlobalLocalization : public rclcpp::Node
{
public:
    GlobalLocalization() : Node("global_localization")
    {
        this->declare_parameter("map_voxel_size", 0.4);
        this->declare_parameter("scan_voxel_size", 0.1);
        this->declare_parameter("freq_localization", 0.5);
        this->declare_parameter("freq_global_map", 0.25);
        this->declare_parameter("localization_threshold", 0.8);
        this->declare_parameter("fov", 6.28319);
        this->declare_parameter("fov_far", 300.0);
        this->declare_parameter("pcd_map_path", "");
        this->declare_parameter("pcd_map_topic", "/map");
        this->declare_parameter("icp_coarse_max_corr_dist", 1.5);
        this->declare_parameter("icp_fine_max_corr_dist", 0.3);
        this->declare_parameter("icp_rmse_threshold", 0.2);

        map_voxel_size_ = this->get_parameter("map_voxel_size").as_double();
        scan_voxel_size_ = this->get_parameter("scan_voxel_size").as_double();
        freq_localization_ = this->get_parameter("freq_localization").as_double();
        localization_threshold_ = this->get_parameter("localization_threshold").as_double();
        fov_ = this->get_parameter("fov").as_double();
        fov_far_ = this->get_parameter("fov_far").as_double();
        pcd_map_path_ = this->get_parameter("pcd_map_path").as_string();
        icp_coarse_max_corr_dist_ = this->get_parameter("icp_coarse_max_corr_dist").as_double();
        icp_fine_max_corr_dist_ = this->get_parameter("icp_fine_max_corr_dist").as_double();
        icp_rmse_threshold_ = this->get_parameter("icp_rmse_threshold").as_double();

        if (!load_global_map()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load global map from: %s",
                        pcd_map_path_.c_str());
            return;
        }

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10,
            std::bind(&GlobalLocalization::cb_scan, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&GlobalLocalization::cb_odom, this, std::placeholders::_1));

        sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&GlobalLocalization::cb_initial_pose, this, std::placeholders::_1));

        pub_map_to_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 10);
        pub_pc_in_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan_in_map", 10);
        pub_submap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 10);

        auto period_ms = static_cast<int64_t>(1.0 / freq_localization_ * 1000.0);
        timer_localization_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&GlobalLocalization::timer_callback, this));

        T_map_to_odom_.setIdentity();
        initialized_ = false;
        scan_received_ = false;

        RCLCPP_INFO(this->get_logger(), "GlobalLocalization node started, map: %ld points",
                    global_map_->size());
    }

private:
    bool load_global_map()
    {
        if (pcd_map_path_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No PCD map path specified");
            return false;
        }

        PointCloudPtr raw_map(new PointCloud);
        if (pcl::io::loadPCDFile<PointType>(pcd_map_path_, *raw_map) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD: %s", pcd_map_path_.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded raw map: %ld points", raw_map->size());

        global_map_ = PointCloudPtr(new PointCloud());
        pcl::VoxelGrid<PointType> vf;
        vf.setInputCloud(raw_map);
        vf.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
        vf.filter(*global_map_);

        RCLCPP_INFO(this->get_logger(), "Downsampled map: %ld points", global_map_->size());
        return true;
    }

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

    static Eigen::Matrix4d inverse_se3(const Eigen::Matrix4d& T)
    {
        Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
        T_inv.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
        T_inv.block<3, 1>(0, 3) = -T.block<3, 3>(0, 0).transpose() * T.block<3, 1>(0, 3);
        return T_inv;
    }

    PointCloudPtr voxel_down_sample(const PointCloudPtr& cloud, double voxel_size)
    {
        PointCloudPtr down(new PointCloud);
        pcl::VoxelGrid<PointType> vf;
        vf.setInputCloud(cloud);
        vf.setLeafSize(static_cast<float>(voxel_size), static_cast<float>(voxel_size),
                       static_cast<float>(voxel_size));
        vf.filter(*down);
        return down;
    }

    struct ICPResult {
        Eigen::Matrix4d transformation;
        double fitness;
        double rmse;
        int inlier_count;
        int point_count;
    };

    ICPResult registration_at_scale(const PointCloudPtr& scan_down,
                                    const PointCloudPtr& map_down,
                                    const Eigen::Matrix4d& initial,
                                    double max_dist)
    {
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setInputTarget(map_down);
        icp.setInputSource(scan_down);
        icp.setMaxCorrespondenceDistance(max_dist);
        icp.setMaximumIterations(20);
        icp.setTransformationEpsilon(1e-6);

        PointCloud aligned;
        icp.align(aligned, initial.cast<float>());

        ICPResult result;
        result.transformation = icp.getFinalTransformation().cast<double>();
        result.rmse = std::numeric_limits<double>::infinity();
        result.inlier_count = 0;
        result.point_count = static_cast<int>(aligned.size());

        // Compute Open3D-style fitness: ratio of source points with a
        // correspondence within max_correspondence_distance (0~1, >0.8 = good)
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(map_down);
        int inlier_count = 0;
        double squared_error_sum = 0.0;
        for (const auto& pt : aligned.points) {
            std::vector<int> indices(1);
            std::vector<float> dists(1);
            if (kdtree.nearestKSearch(pt, 1, indices, dists) > 0) {
                if (std::sqrt(dists[0]) < max_dist) {
                    inlier_count++;
                    squared_error_sum += dists[0];
                }
            }
        }
        result.inlier_count = inlier_count;
        result.fitness = (aligned.size() > 0)
            ? static_cast<double>(inlier_count) / static_cast<double>(aligned.size())
            : 0.0;
        if (inlier_count > 0) {
            result.rmse = std::sqrt(squared_error_sum / static_cast<double>(inlier_count));
        }
        return result;
    }

    PointCloudPtr crop_global_map_in_fov(const Eigen::Matrix4d& pose_estimation)
    {
        Eigen::Matrix4d T_odom_to_baselink = pose_to_mat(cur_odom_->pose.pose);
        Eigen::Matrix4d T_map_to_baselink = pose_estimation * T_odom_to_baselink;
        Eigen::Matrix4d T_baselink_to_map = inverse_se3(T_map_to_baselink);

        // transform global map points to baselink frame
        PointCloudPtr map_in_baselink(new PointCloud);
        pcl::transformPointCloud(*global_map_, *map_in_baselink,
                                 T_baselink_to_map.cast<float>());

        // filter by FOV
        PointCloudPtr map_in_fov(new PointCloud);
        double half_fov = fov_ / 2.0;
        for (const auto& pt : map_in_baselink->points) {
            if (std::isnan(pt.x) || std::isnan(pt.y)) continue;
            double x = pt.x, y = pt.y;
            if (fov_ > 3.14) {
                // 360 deg LiDAR: no forward check, just distance + angle
                if (x < fov_far_ && std::abs(std::atan2(y, x)) < half_fov) {
                    map_in_fov->points.push_back(pt);
                }
            } else {
                // forward FOV
                if (x > 0 && x < fov_far_ && std::abs(std::atan2(y, x)) < half_fov) {
                    map_in_fov->points.push_back(pt);
                }
            }
        }
        map_in_fov->width = map_in_fov->size();
        map_in_fov->height = 1;
        map_in_fov->is_dense = true;

        // transform back to map frame for submap debug publish
        PointCloudPtr submap_in_map(new PointCloud);
        pcl::transformPointCloud(*map_in_fov, *submap_in_map,
                                 T_map_to_baselink.cast<float>());

        // publish submap (downsample 10x for debug)
        publish_point_cloud(pub_submap_, submap_in_map, cur_odom_->header, 10);

        // return the map in FOV but in map frame for ICP target
        // Actually, ICP needs target in same frame as the initial transform expects.
        // The scan is in its own frame (body), initial is map_to_odom.
        // We need the map points in map frame, cropped based on FOV.
        // So return map points in map frame that are within FOV.
        // Let's re-derive: we want map points that are visible from current pose.
        // We already have indices in baselink frame. Let's get the original map points.
        PointCloudPtr map_in_map_fov(new PointCloud);
        // re-scan to collect original map points with same indices
        // Simpler: just crop by bounding box in map frame using position
        // Actually let's just use the submap_in_map we already have (points back in map frame)
        // But we downsampled 10x for publish. For ICP we want full resolution FOV crop.
        // Let's redo: get indices from baselink transform, then pick from global_map_

        // Collect indices
        std::vector<int> indices;
        int idx = 0;
        for (const auto& pt : map_in_baselink->points) {
            if (std::isnan(pt.x) || std::isnan(pt.y)) { idx++; continue; }
            double x = pt.x, y = pt.y;
            bool in_fov = false;
            if (fov_ > 3.14) {
                in_fov = (x < fov_far_ && std::abs(std::atan2(y, x)) < half_fov);
            } else {
                in_fov = (x > 0 && x < fov_far_ && std::abs(std::atan2(y, x)) < half_fov);
            }
            if (in_fov) indices.push_back(idx);
            idx++;
        }

        map_in_map_fov->points.reserve(indices.size());
        for (int i : indices) {
            map_in_map_fov->points.push_back(global_map_->points[i]);
        }
        map_in_map_fov->width = map_in_map_fov->size();
        map_in_map_fov->height = 1;
        map_in_map_fov->is_dense = true;

        return map_in_map_fov;
    }

    void global_localization(const Eigen::Matrix4d& pose_estimation)
    {
        auto scan_copy = cur_scan_raw_;

        if (!scan_copy || scan_copy->size() < 100) {
            RCLCPP_WARN(this->get_logger(), "Scan too small (%ld pts), skip ICP",
                        scan_copy ? scan_copy->size() : 0);
            return;
        }

        PointCloudPtr map_in_fov = crop_global_map_in_fov(pose_estimation);

        if (map_in_fov->empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "FOV crop returned 0 points, falling back to full map (%ld)",
                        global_map_->size());
            map_in_fov = global_map_;
        }

        if (map_in_fov->size() < 50) {
            RCLCPP_WARN(this->get_logger(), "Map too small (%ld pts), skip ICP",
                        map_in_fov->size());
            return;
        }

        // Diagnostic: show bounding boxes
        auto compute_bbox = [](const PointCloudPtr& cloud) -> std::string {
            float mx = -1e9, Mx = 1e9, my = -1e9, My = 1e9, mz = -1e9, Mz = 1e9;
            mx = my = mz = 1e9; Mx = My = Mz = -1e9;
            for (const auto& p : cloud->points) {
                if (std::isnan(p.x)) continue;
                mx = std::min(mx, p.x); Mx = std::max(Mx, p.x);
                my = std::min(my, p.y); My = std::max(My, p.y);
                mz = std::min(mz, p.z); Mz = std::max(Mz, p.z);
            }
            char buf[256];
            snprintf(buf, sizeof(buf), "x[%.1f~%.1f] y[%.1f~%.1f] z[%.1f~%.1f]", mx, Mx, my, My, mz, Mz);
            return std::string(buf);
        };
        Eigen::Vector3d init_t = pose_estimation.block<3, 1>(0, 3);
        // RCLCPP_INFO(this->get_logger(),
        // "ICP diag: scan[%ld] %s | map[%ld] %s | init_t(%.2f,%.2f,%.2f) odom_t(%.2f,%.2f,%.2f)",
        //     scan_copy->size(), compute_bbox(scan_copy).c_str(),
        //     map_in_fov->size(), compute_bbox(map_in_fov).c_str(),
        //     init_t.x(), init_t.y(), init_t.z(),
        //     cur_odom_ ? cur_odom_->pose.pose.position.x : 0.0,
        //     cur_odom_ ? cur_odom_->pose.pose.position.y : 0.0,
        //     cur_odom_ ? cur_odom_->pose.pose.position.z : 0.0);

        auto scan_ds5 = voxel_down_sample(scan_copy, scan_voxel_size_ * 5);
        auto map_ds5 = voxel_down_sample(map_in_fov, map_voxel_size_ * 5);

        auto coarse = registration_at_scale(scan_ds5, map_ds5, pose_estimation, icp_coarse_max_corr_dist_);

        auto scan_ds1 = voxel_down_sample(scan_copy, scan_voxel_size_ * 1);
        auto map_ds1 = voxel_down_sample(map_in_fov, map_voxel_size_ * 1);

        auto fine = registration_at_scale(scan_ds1, map_ds1, coarse.transformation, icp_fine_max_corr_dist_);

        RCLCPP_INFO(this->get_logger(),
                    "ICP fitness coarse=%.2f%% fine=%.2f%% threshold=%.2f%% | fine rmse=%.3fm threshold=%.3fm | inliers=%d/%d",
                    coarse.fitness * 100.0,
                    fine.fitness * 100.0,
                    localization_threshold_ * 100.0,
                    fine.rmse,
                    icp_rmse_threshold_,
                    fine.inlier_count,
                    fine.point_count);

        if (fine.fitness < localization_threshold_ || fine.rmse > icp_rmse_threshold_) {
            RCLCPP_WARN(this->get_logger(),
                        "ICP rejected: fine fitness %.2f%% / rmse %.3fm did not pass %.2f%% / %.3fm. Set the initial pose again closer to the real robot pose.",
                        fine.fitness * 100.0,
                        fine.rmse,
                        localization_threshold_ * 100.0,
                        icp_rmse_threshold_);
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "ICP accepted: fine fitness %.2f%% >= %.2f%% and rmse %.3fm <= %.3fm. Localization active, publishing /map_to_odom.",
                    fine.fitness * 100.0,
                    localization_threshold_ * 100.0,
                    fine.rmse,
                    icp_rmse_threshold_);
        T_map_to_odom_ = fine.transformation;
        publish_odom(fine.transformation);
    }

    void publish_odom(const Eigen::Matrix4d& transform)
    {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        Eigen::Vector3d t = transform.block<3, 1>(0, 3);
        msg.pose.pose.position.x = t.x();
        msg.pose.pose.position.y = t.y();
        msg.pose.pose.position.z = t.z();

        Eigen::Quaterniond q(transform.block<3, 3>(0, 0));
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        pub_map_to_odom_->publish(msg);
    }

    void publish_point_cloud(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
        const PointCloudPtr& cloud,
        const std_msgs::msg::Header& header,
        int stride)
    {
        if (pub->get_subscription_count() == 0) return;

        PointCloudPtr to_publish(new PointCloud);
        if (stride > 1) {
            to_publish->points.reserve(cloud->size() / stride);
            for (size_t i = 0; i < cloud->size(); i += stride) {
                to_publish->points.push_back(cloud->points[i]);
            }
            to_publish->width = to_publish->size();
            to_publish->height = 1;
            to_publish->is_dense = true;
        } else {
            to_publish = cloud;
        }

        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(*to_publish, out);
        out.header = header;
        out.header.frame_id = "map";
        pub->publish(out);
    }

    void cb_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cur_scan_raw_ = PointCloudPtr(new PointCloud());
        pcl::fromROSMsg(*msg, *cur_scan_raw_);
        cur_scan_header_ = msg->header;
        scan_received_ = true;

        // publish cur_scan_in_map for debug (transform by T_map_to_odom)
        // In Python: just republishes the raw scan with its original header
        // The Python code publishes raw pc without transform
        sensor_msgs::msg::PointCloud2 out;
        out = *msg; // just forward
        pub_pc_in_map_->publish(out);
    }

    void cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cur_odom_ = msg;
    }

    void cb_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        Eigen::Matrix4d initial_map_to_odom;
        bool have_scan = false;

        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (!cur_odom_) {
                RCLCPP_WARN(this->get_logger(),
                            "Initial pose ignored: no /Odometry received yet");
                return;
            }

            // RViz publishes T_map_to_base, while ICP aligns a scan already
            // expressed in the FAST-LIO odom (camera_init) frame.  Therefore
            // the ICP initial guess must be T_map_to_odom, not T_map_to_base.
            const Eigen::Matrix4d T_map_to_base = pose_to_mat(msg->pose.pose);
            const Eigen::Matrix4d T_odom_to_base = pose_to_mat(cur_odom_->pose.pose);
            initial_map_to_odom = T_map_to_base * inverse_se3(T_odom_to_base);

            T_map_to_odom_ = initial_map_to_odom;
            initialized_ = true;
            have_scan = scan_received_;
            RCLCPP_INFO(this->get_logger(),
                        "Initial pose received; computed map->odom from current odometry");
        }

        if (have_scan) {
            do_localization(initial_map_to_odom);
        }
    }

    void timer_callback()
    {
        if (!initialized_) return;

        Eigen::Matrix4d pose_est;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!scan_received_) return;
            pose_est = T_map_to_odom_;
        }

        do_localization(pose_est);
    }

    void do_localization(const Eigen::Matrix4d& pose_estimation)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!cur_scan_raw_ || cur_scan_raw_->empty()) return;
        if (!cur_odom_) return;

        global_localization(pose_estimation);
    }

    // Parameters
    double map_voxel_size_;
    double scan_voxel_size_;
    double freq_localization_;
    double localization_threshold_;
    double icp_coarse_max_corr_dist_;
    double icp_fine_max_corr_dist_;
    double icp_rmse_threshold_;
    double fov_;
    double fov_far_;
    std::string pcd_map_path_;
    std::string pcd_map_topic_;

    // Global map
    PointCloudPtr global_map_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_to_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_in_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_localization_;

    // State
    nav_msgs::msg::Odometry::SharedPtr cur_odom_;
    PointCloudPtr cur_scan_raw_;
    std_msgs::msg::Header cur_scan_header_;
    bool initialized_;
    bool scan_received_;
    Eigen::Matrix4d T_map_to_odom_;

    std::mutex mutex_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalLocalization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
