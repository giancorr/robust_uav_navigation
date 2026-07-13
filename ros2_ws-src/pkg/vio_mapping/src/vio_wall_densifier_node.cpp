#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

class VioWallDensifierNode : public rclcpp::Node
{
public:
    VioWallDensifierNode() : Node("vio_wall_densifier_node")
    {
        this->declare_parameter<double>("max_wall_length", 3.0);
        max_wall_length_ = this->get_parameter("max_wall_length").as_double();

        // Subscribe to the original OpenVINS sparse pointcloud
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ov_msckf/points_slam", 10,
            std::bind(&VioWallDensifierNode::pointcloud_callback, this, std::placeholders::_1));

        // Publish the enriched/densified pointcloud
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/ov_msckf/points_slam_densified", 10);

        // Subscribe to odometry to know where the drone is
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ov_msckf/odomimu", 10,
            std::bind(&VioWallDensifierNode::odometry_callback, this, std::placeholders::_1));

        this->declare_parameter<double>("max_drone_distance", 5.0);
        max_drone_distance_ = this->get_parameter("max_drone_distance").as_double();

        this->declare_parameter<double>("cluster_tolerance", 0.5);
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();

        RCLCPP_INFO(this->get_logger(), "VIO Wall Densifier Node initialized (max_wall_length=%.1fm, max_drone_distance=%.1fm, cluster_tolerance=%.2fm).", 
                    max_wall_length_, max_drone_distance_, cluster_tolerance_);
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_drone_pos_ = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        has_odom_ = true;
    }
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // Apply Thesis Logic: VIO Recovery / Wall Densification
        pcl::PointCloud<pcl::PointXYZ>::Ptr densified_cloud = apply_wall_densification(cloud);

        // Convert back to ROS PointCloud2 and publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*densified_cloud, output_msg);
        
        // Preserve original header (frame_id and stamp are critical for vio_mapping!)
        output_msg.header = msg->header;

        pointcloud_pub_->publish(output_msg);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_wall_densification(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        
        // 1. Keep all original points
        *output_cloud = *input_cloud;

        // If we don't have odometry yet, we can't filter by distance
        if (!has_odom_) {
            return output_cloud;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr nearby_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& pt : input_cloud->points) {
            double dist = std::sqrt(std::pow(pt.x - current_drone_pos_.x(), 2) + 
                                    std::pow(pt.y - current_drone_pos_.y(), 2) + 
                                    std::pow(pt.z - current_drone_pos_.z(), 2));
            if (dist <= max_drone_distance_) {
                nearby_cloud->points.push_back(pt);
            }
        }

        if (nearby_cloud->points.size() < 10) {
            return output_cloud; // Too few nearby points to find a wall
        }

        // Apply Euclidean Clustering to separate physically distinct objects (e.g. left vs right wall)
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(nearby_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_); // parameter for clustering distance
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(nearby_cloud);
        ec.extract(cluster_indices);

        int num_walls_found = 0;

        for (const auto& cluster : cluster_indices) {
            if (num_walls_found >= 2) break; // Limit to 2 walls max

            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            for (const auto& idx : cluster.indices) {
                cluster_cloud->points.push_back(nearby_cloud->points[idx]);
            }
            cluster_cloud->width = cluster_cloud->points.size();
            cluster_cloud->height = 1;
            cluster_cloud->is_dense = true;

            pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            *remaining_cloud = *cluster_cloud;

            // Setup RANSAC plane segmentation for this specific cluster
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(100);
            seg.setDistanceThreshold(0.20);

            pcl::ExtractIndices<pcl::PointXYZ> extract;

            // Find the dominant plane in this cluster
            seg.setInputCloud(remaining_cloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0) {
                continue; // No plane found in this cluster
            }

            // Check if the plane is vertical (a wall). 
            double a = coefficients->values[0];
            double b = coefficients->values[1];
            double c = coefficients->values[2];
            double d = coefficients->values[3];
            
            if (std::abs(c) < 0.3) { // It's a vertical wall!
                
                pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                extract.setInputCloud(remaining_cloud);
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter(*wall_cloud);

                pcl::PointXYZ min_pt, max_pt;
                pcl::getMinMax3D(*wall_cloud, min_pt, max_pt);

                double wall_span_x = max_pt.x - min_pt.x;
                double wall_span_y = max_pt.y - min_pt.y;
                double wall_span = std::max(wall_span_x, wall_span_y);
                
                if (wall_span > max_wall_length_) {
                    RCLCPP_DEBUG(this->get_logger(), "Skipping wall densification: span %.1fm > max %.1fm", wall_span, max_wall_length_);
                } else {
                    double step = 0.2;
                    
                    if (std::abs(a) > std::abs(b)) {
                        for (double y = min_pt.y; y <= max_pt.y; y += step) {
                            for (double z = min_pt.z; z <= max_pt.z + 1.0; z += step) {
                                double x = -(b * y + c * z + d) / a;
                                output_cloud->points.push_back(pcl::PointXYZ(x, y, z));
                            }
                        }
                    } else {
                        for (double x = min_pt.x; x <= max_pt.x; x += step) {
                            for (double z = min_pt.z; z <= max_pt.z + 1.0; z += step) {
                                double y = -(a * x + c * z + d) / b;
                                output_cloud->points.push_back(pcl::PointXYZ(x, y, z));
                            }
                        }
                    }
                }
                
                num_walls_found++;
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1; // Unorganized point cloud
        output_cloud->is_dense = true;

        return output_cloud;
    }

    double max_wall_length_;
    double max_drone_distance_;
    double cluster_tolerance_;
    Eigen::Vector3d current_drone_pos_ = Eigen::Vector3d::Zero();
    bool has_odom_ = false;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VioWallDensifierNode>());
    rclcpp::shutdown();
    return 0;
}
