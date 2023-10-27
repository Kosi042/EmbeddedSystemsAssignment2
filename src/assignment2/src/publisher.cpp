#include <chrono>
#include <functional>
#include <string>
#include <memory>
#include <iostream>
#include <vector>

#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp> 

using namespace std::chrono_literals;

#define FLATTENED_MINIMIYED_IMAGE 200 * 300 * 3

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("img_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      sensor_msgs::msg::Image message;
      message.data = std::vector<unsigned char>(fromFpga);
      message.height = 200;
      message.width = 300;

      publisher_->publish(message);
    }

    unsigned char fromFpga;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}