#include "minimal_publisher.hpp"

MinimalPublisher::MinimalPublisher ()
: Node("minimal_publisher"), count_(0)
{  
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),  // Use std::chrono to specify 500ms
        std::bind(&MinimalPublisher::timer_callback, this)
    );
}

void MinimalPublisher::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
 }

 MinimalPublisher::~MinimalPublisher(){}