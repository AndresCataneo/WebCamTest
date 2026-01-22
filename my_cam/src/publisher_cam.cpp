#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class PublisherCamNode : public rclcpp::Node
{
public:
    PublisherCamNode()
    : Node("publisher_cam")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_raw", rclcpp::SensorDataQoS());

        cap_.open(0);  // Webcam por defecto

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la webcam");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Webcam abierta correctamente");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // ~30 FPS
            std::bind(&PublisherCamNode::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Frame vacío");
            return;
        }

        // OpenCV usa BGR por defecto
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            frame
        ).toImageMsg();

        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";

        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherCamNode>());
    rclcpp::shutdown();
    return 0;
}
