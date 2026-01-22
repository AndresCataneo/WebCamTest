#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <iostream>

class SubscriberCamNode : public rclcpp::Node
{
public:
    SubscriberCamNode() : Node("subscriber_cam"), count_(0)
    {
        // Crear la suscripción ROS 2 puro (sin image_transport)
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw",  // Nombre del topic
            10,           // Profundidad de la cola
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                count_++;
                if (count_ % 30 == 0) {
                    RCLCPP_INFO(this->get_logger(),
                              "Imagen recibida #%d: %dx%d, encoding: %s, %zu bytes",
                              count_,
                              msg->width, 
                              msg->height, 
                              msg->encoding.c_str(),
                              msg->data.size());
                    
                    // Mostrar un pixel de ejemplo
                    if (!msg->data.empty()) {
                        RCLCPP_INFO(this->get_logger(),
                                  "Pixel(0,0): R=%d, G=%d, B=%d",
                                  msg->data[0], msg->data[1], msg->data[2]);
                    }
                }
            });
        
        RCLCPP_INFO(this->get_logger(), "Suscriptor iniciado. Esperando imágenes en /image_raw");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberCamNode>());
    rclcpp::shutdown();
    return 0;
}