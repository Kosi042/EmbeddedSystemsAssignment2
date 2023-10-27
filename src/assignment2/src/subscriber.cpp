#include <memory>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp> //use image.hpp dont know if correct
using std::placeholders::_1;

#define FLATTENED_MINIMIYED_IMAGE 200 * 300 * 3

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("img_subscriber")
    , toFpga{0}
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
      auto storage = std::vector<uchar>(FLATTENED_MINIMIYED_IMAGE); // or however many channels you have
      auto mat     = cv::Mat_<uchar>(200, 300, CV_8UC3, storage.data());
      auto tmp = vector<uchar>(img.begin<uchar>(), img.end<uchar>());
      toFpga = &tmp[0];


    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    unsigned char toFpga[FLATTENED_MINIMIYED_IMAGE];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
