cmake_minimum_required(VERSION 3.8)
project(my_cam)

set(CMAKE_CXX_STANDARD 17)

# Buscar dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Publicador
add_executable(publisher_cam src/publisher_cam.cpp)
ament_target_dependencies(publisher_cam
    rclcpp
    sensor_msgs
    cv_bridge
    image_transport
)

# Suscriptor
add_executable(subscriber_cam src/subscriber_cam.cpp)
ament_target_dependencies(subscriber_cam
    rclcpp
    sensor_msgs
    cv_bridge
    image_transport
)

# Instalar los ejecutables
install(TARGETS
    publisher_cam
    subscriber_cam
    DESTINATION lib/${PROJECT_NAME}
)

# Instalación de recursos y otros archivos si es necesario
ament_package()
