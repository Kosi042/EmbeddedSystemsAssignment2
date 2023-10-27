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
      "/image_raw", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
	  cv::Mat img = cv_ptr->image;
      cv::Mat img_small;
      resize(img, img_small, cv::Size(300, 200), cv::INTER_LINEAR);
      std::cout << "H: " << img_small.rows << "W: " << img_small.cols << std::endl;
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
