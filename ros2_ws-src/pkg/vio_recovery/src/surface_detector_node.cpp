#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vio_recovery/msg/surface_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

// Surface characterization
struct SurfaceData {
    bool valid = false;     // If it is in a correct range
    double distance = -1.0; // Distance from depth camera
    double angle_deg = 0.0; // Angle observed in depth camera
    std::string name = "";  // Left, Right or Center (divide image plane in three parts)
};

class SurfaceDetector : public rclcpp::Node {
public:
    SurfaceDetector() : Node("surface_detector_node") {
        
        // Params, Subscribers and Publishers
        this->declare_parameter<double>("min_valid_dist", 0.2);
        this->declare_parameter<double>("max_valid_dist", 2.5);
        this->declare_parameter<double>("camera_fx", 432.61);
        
        this->get_parameter("min_valid_dist", min_valid_dist_);
        this->get_parameter("max_valid_dist", max_valid_dist_);
        this->get_parameter("camera_fx", focal_x_);
        
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth_camera", 10,
            std::bind(&SurfaceDetector::depth_callback, this, std::placeholders::_1));

        surface_pub_ = this->create_publisher<vio_recovery::msg::SurfaceInfo>("/surface_info", 10);
    }

private:
    double min_valid_dist_;
    double max_valid_dist_;
    double focal_x_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<vio_recovery::msg::SurfaceInfo>::SharedPtr surface_pub_;

    // Median depth computation
    double get_median_depth(const cv::Mat& roi) {
        std::vector<float> valid_depths;
        valid_depths.reserve(roi.rows * roi.cols);

        for (int r = 0; r < roi.rows; ++r) {
            for (int c = 0; c < roi.cols; ++c) {
                float d = roi.at<float>(r, c);
                // Save only finite distance pixels
                if (std::isfinite(d) && d >= min_valid_dist_ && d <= max_valid_dist_) {
                    valid_depths.push_back(d);
                }
            }
        }

        // If in the ROI there is empty, return not valid
        if (valid_depths.empty()) return -1.0;

        // Compute median
        std::nth_element(valid_depths.begin(), valid_depths.begin() + valid_depths.size() / 2, valid_depths.end());
        return static_cast<double>(valid_depths[valid_depths.size() / 2]);
    }

    // Sector analysis
    SurfaceData analyze_sector(const cv::Mat& img, const cv::Rect& bounds, const std::string& name) {
        SurfaceData data;
        data.name = name;
        
        cv::Mat full_roi = img(bounds);
        data.distance = get_median_depth(full_roi);

        // If there is a valid surface in the sector compute the angle
        if (data.distance > 0) {
            data.valid = true;

            int half_width = bounds.width / 2;
            cv::Rect left_sub_rect(bounds.x, bounds.y, half_width, bounds.height);
            cv::Rect right_sub_rect(bounds.x + half_width, bounds.y, half_width, bounds.height);

            double z_left = get_median_depth(img(left_sub_rect));
            double z_right = get_median_depth(img(right_sub_rect));

            if (z_left > 0 && z_right > 0) {
                double delta_z = z_right - z_left;
                double avg_z = (z_left + z_right) / 2.0;
                
                double delta_u_pixels = half_width; 
                double delta_x = (delta_u_pixels * avg_z) / focal_x_;

                double wall_angle_rad = std::atan2(delta_z, delta_x);
                data.angle_deg = wall_angle_rad * (180.0 / M_PI);
            }
        }
        return data;
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Get image from OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat depth_img = cv_ptr->image;

            int band_height = 100;
            int start_y = (depth_img.rows / 2) - (band_height / 2);
            
            // Divide image into left, center and right
            int third_width = depth_img.cols / 3;

            cv::Rect rect_left(0, start_y, third_width, band_height);
            cv::Rect rect_center(third_width, start_y, third_width, band_height);
            cv::Rect rect_right(third_width * 2, start_y, third_width, band_height);

            // Extract data
            SurfaceData s_left = analyze_sector(depth_img, rect_left, "LEFT");
            SurfaceData s_center = analyze_sector(depth_img, rect_center, "CENTER");
            SurfaceData s_right = analyze_sector(depth_img, rect_right, "RIGHT");

            // Select closest surface 
            SurfaceData closest = s_center; 
            if (!closest.valid) closest.distance = 999.0;
            if (s_left.valid && s_left.distance < closest.distance) closest = s_left;
            if (s_right.valid && s_right.distance < closest.distance) closest = s_right;

            // Publish message
            vio_recovery::msg::SurfaceInfo surface_msg;
            
            // Pack LEFT data
            surface_msg.dist_left = s_left.valid ? s_left.distance : -1.0;
            surface_msg.angle_left = s_left.valid ? s_left.angle_deg : 0.0;
            
            // Pack CENTER data
            surface_msg.dist_center = s_center.valid ? s_center.distance : -1.0;
            surface_msg.angle_center = s_center.valid ? s_center.angle_deg : 0.0;
            
            // Pack RIGHT data
            surface_msg.dist_right = s_right.valid ? s_right.distance : -1.0;
            surface_msg.angle_right = s_right.valid ? s_right.angle_deg : 0.0;

            // Pack closest sector
            surface_msg.closest_sector = closest.valid ? closest.name : "NONE";

            surface_pub_->publish(surface_msg);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SurfaceDetector>());
    rclcpp::shutdown();
    return 0;
}