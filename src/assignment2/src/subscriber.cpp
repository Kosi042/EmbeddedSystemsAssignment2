#include <memory>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "drivers/ximge_processor.h" // Include the header for your XImge_processor

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
	MinimalSubscriber()
		: Node("img_subscriber")
	{
		publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);

		subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/image_raw", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

		// Initialize the XImge_processor
		int status = XImge_processor_Initialize(&ip_inst, "imge_processor");
		if (status != XST_SUCCESS) {
			RCLCPP_ERROR(this->get_logger(), "Error: Could not initialize the IP core.");
			return;
		}

		// Resize the output image to match your requirements
		out_image_.height = 100;
		out_image_.width = 150;
		out_image_.encoding = "bgr8"; // Update with the actual encoding
		out_image_.step = 150 * 3; // Assuming 3 channels
		out_image_.data.resize(out_image_.height * out_image_.step);
	}

	~MinimalSubscriber()
	{
		XImge_processor_Release(&ip_inst);
	}

private:
	void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
		cv::Mat img = cv_ptr->image;
		
		cv::Mat img_small;
		resize(img, img_small, cv::Size(150, 100), cv::INTER_LINEAR);

		std::cout << "H: " << img_small.rows << " W: " << img_small.cols << std::endl;

		// Perform processing on img_small

		// For the sake of demonstration, let's assume you copy the processed image to out_image_
		std::memcpy(out_image_.data.data(), img_small.data, out_image_.data.size());

		// Publish the processed image
		publisher_->publish(out_image_);
	}

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	sensor_msgs::msg::Image out_image_;

	XImge_processor ip_inst;
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalSubscriber>());
	rclcpp::shutdown();
	return 0;
}

