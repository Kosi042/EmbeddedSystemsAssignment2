#include <memory>
#include <iostream>

#include <cv_bridge/cv_bridge.h> 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp> //use image.hpp dont know if correct
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      std::cout << "H: " << msg->height << "W: " << msg->width << std::endl;
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
	  cv::Mat img = cv_ptr->image;
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}