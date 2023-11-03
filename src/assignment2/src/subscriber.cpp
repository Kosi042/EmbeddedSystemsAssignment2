#include <memory>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "drivers/ximge_processor.h" // Include the header for your XImge_processor

using std::placeholders::_1;
using namespace std::chrono_literals;

#define DATA_SIZE 16*16

class MinimalSubscriber
	: public rclcpp::Node
{
public:
	MinimalSubscriber()
		: Node("img_subscriber")
	{
		// Create a publisher that will publish the filtered image with frequency of 10Hz.
		publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);

		// Create a subscriber to the topic created by the camera node.
		subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/image_raw", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

		// Initialize the XImge_processor the name is obtained by 'cat /sys/class/uio/uiox/name'
		int status = XImge_processor_Initialize(&ip_inst, "imge_processor");
		if (status != XST_SUCCESS) {
			RCLCPP_ERROR(this->get_logger(), "Error: Could not initialize the IP core.");
			return;
		}

		// Resize the output image to match your requirements
		out_image_.height = 24;
		out_image_.width = 32;
		out_image_.encoding = "mono8"; // Update with the actual encoding
	}

	~MinimalSubscriber()
	{
		XImge_processor_Release(&ip_inst);
	}

private:
	/**
	 * @brief Reshapes one dimensional input vector to cv::Mat of size (rows, cols).
	 *
	 * @param inputVector Data to be reshaped.
	 * @param rows Number of rows in the output matrix.
	 * @param cols Number of columns in the output matrix.
	 * @return The reshaped matrix containing data from inputVector.
	 */
	cv::Mat vectorToMat(const std::vector<uint32_t>& inputVector, int rows, int cols) {
		cv::Mat image(rows, cols, CV_8UC1);

		if (static_cast<int>(inputVector.size()) != rows * cols) {
			std::cerr << "Input vector size does not match the specified rows and columns." << std::endl;
			return cv::Mat(); // Return an empty Mat in case of an error.
		}

		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				uint32_t pixelValue = inputVector[i * cols + j] & 0xFF; // Extract the least significant byte.
				image.at<uchar>(i, j) = static_cast<uchar>(pixelValue);
			}
		}

		return image;
	}

	void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		// convert ros Image to cv Image.
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
		cv::Mat img = cv_ptr->image;

		// Resize the image from 640x420 to 32x24.
		cv::Mat img_small;
		resize(img, img_small, cv::Size(32, 24));

		std::cout << "H: " << img_small.rows << " W: " << img_small.cols << std::endl;

		// Convert the RGB picture to Grayscale picture.
		cv::Mat grayscaleImage;
		cv::cvtColor(img_small, grayscaleImage, cv::COLOR_BGR2GRAY);

		// Reshape data from 2D representation to 1D vector.
		cv::Mat flattenedVector = grayscaleImage.reshape(1, 1);
		std::vector<word_type> flattenedStdVector;
		flattenedStdVector.assign(flattenedVector.data, flattenedVector.data + flattenedVector.total());
		word_type *data = flattenedStdVector.data();

		// Write data to the HLS generated IP.
		XImge_processor_Write_in_r_Words(ip_inst, 0, data, flattenedStdVector.size());

		// Start the execution of the IP block.
		XImge_processor_Start(&ip_inst);

		// Wate until the data are processed.
		while (!XImge_processor_IsDone(&ip_inst));

		// Read the input from the IP.
		XImge_processor_Read_out_r_Words(ip_inst, 0, data, DATA_SIZE);

		// Convert the read data to 32x24 cv::Mat Image using the function vectorToMat.
		auto filteredImg = vectorToMat(flattenedStdVector, 32, 24);

		// Create ros Image from cv Image.
		out_image_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", filteredImg)

		// Publish the processed image
		publisher_->publish(out_image_);
	}

	// Subscriber to the /image_raw topic that the camera node created.
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

	// Publisher publishing the filtered image on topic /processed_image.
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

	// Output image that is sent to the publisher_.
	sensor_msgs::msg::Image out_image_;

	// XImge_processor that supports the communication with IP.
	XImge_processor ip_inst;
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalSubscriber>());
	rclcpp::shutdown();
	return 0;
}

