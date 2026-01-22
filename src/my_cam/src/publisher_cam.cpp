#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <memory>

class PublisherCamNode : public rclcpp::Node
{
public:
    PublisherCamNode() : Node("publisher_cam"), count_(0)
    {
        // Crear publicador ROS 2 puro (sin image_transport)
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        
        RCLCPP_INFO(this->get_logger(), "Publicador iniciado. Topic: /image_raw");
        
        // Temporizador para publicar imágenes de prueba
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 FPS
            std::bind(&PublisherCamNode::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        
        // Crear una imagen de prueba (patrón animado)
        int width = 640;
        int height = 480;
        
        msg->height = height;
        msg->width = width;
        msg->encoding = "rgb8";
        msg->is_bigendian = false;
        msg->step = width * 3;  // 3 bytes por pixel (RGB)
        
        // Reservar espacio para los datos
        msg->data.resize(height * width * 3);
        
        // Llenar con un patrón animado
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = (y * width + x) * 3;
                
                // Patrón RGB animado
                msg->data[index] = static_cast<uint8_t>((x + count_) % 256);      // R
                msg->data[index + 1] = static_cast<uint8_t>((y + count_) % 256);  // G
                msg->data[index + 2] = static_cast<uint8_t>((x + y + count_) % 256);  // B
            }
        }
        
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame_" + std::to_string(count_);
        
        publisher_->publish(*msg);
        
        count_++;
        if (count_ % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "Publicada imagen %d", count_);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherCamNode>());
    rclcpp::shutdown();
    return 0;
}