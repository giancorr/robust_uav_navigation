#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vio_health_monitor/msg/feature_count.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

class FeatureCounterNode : public rclcpp::Node {
public:
    FeatureCounterNode() : Node("feature_counter_node"),
                           front_left_(0), front_right_(0),
                           rear_left_(0), rear_right_(0) {
        
        sub_front_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10,
            std::bind(&FeatureCounterNode::front_callback, this, std::placeholders::_1));

        sub_rear_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/cameraread", 10,
            std::bind(&FeatureCounterNode::rear_callback, this, std::placeholders::_1));

        feature_pub_ = this->create_publisher<vio_health_monitor::msg::FeatureCount>(
            "/vio_help/feature_distribution", 10);
            
        feature_detector_ = cv::FastFeatureDetector::create(20);
    }

private:
    // Count features in front camera
    void front_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat gray;
            cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

            std::vector<cv::KeyPoint> keypoints;
            feature_detector_->detect(gray, keypoints);

            int left_count = 0;
            int right_count = 0;
            int mid_x = gray.cols / 2;

            // Separate feature in left side and right side
            for (const auto& kp : keypoints) {
                if (kp.pt.x < mid_x) left_count++;
                else right_count++;
            }

            front_left_ = left_count;
            front_right_ = right_count;
            publish_totals();

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge error (front): %s", e.what());
        }
    }

    // Count features in rear camera
    void rear_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat gray;
            cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

            std::vector<cv::KeyPoint> keypoints;
            feature_detector_->detect(gray, keypoints);

            int left_count = 0;
            int right_count = 0;
            int mid_x = gray.cols / 2;

            // Separate feature in left side and right side
            for (const auto& kp : keypoints) {
                if (kp.pt.x < mid_x) right_count++; 
                else left_count++;
            }

            rear_left_ = left_count;
            rear_right_ = right_count;
            publish_totals();

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge error (rear): %s", e.what());
        }
    }

    // Publish sum on left side and right side
    void publish_totals() {
        auto pub_msg = vio_health_monitor::msg::FeatureCount();
        pub_msg.left_count = front_left_ + rear_left_;
        pub_msg.right_count = front_right_ + rear_right_;
        feature_pub_->publish(pub_msg);
    }

    int front_left_, front_right_;
    int rear_left_, rear_right_;

    cv::Ptr<cv::FastFeatureDetector> feature_detector_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_front_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rear_;
    rclcpp::Publisher<vio_health_monitor::msg::FeatureCount>::SharedPtr feature_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeatureCounterNode>());
    rclcpp::shutdown();
    return 0;
}