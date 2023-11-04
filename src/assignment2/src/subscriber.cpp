#include <cstdlib>
#include <memory>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

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
		// Initialize the XImge_processor the name is obtained by 'cat /sys/class/uio/uiox/name'
		int status = XImge_processor_Initialize(&ip_inst, "imge_processor");
		if (status != XST_SUCCESS) {
			RCLCPP_ERROR(this->get_logger(), "Error: Could not initialize the IP core.");
			return;
		}
		std::cout << "Imge processor initiated\n";


		// Create a publisher that will publish the filtered image with frequency of 10Hz.
		publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);

		// Create a subscriber to the topic created by the camera node.
		subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/image_raw", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

		// Resize the output image to match your requirements
		// out_image_.height = 16;
		// out_image_.width = 16;
		// out_image_.encoding = "brg8"; // Update with the actual encoding
	}

	~MinimalSubscriber()
	{
		// XImge_processor_Release(&ip_inst);
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
	cv::Mat vectorToMat(const std::vector<int>& inputVector, int rows, int cols) {
		cv::Mat image(rows, cols, CV_8UC1);

		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				// std::cout << i*j+j << " -> " << (int)inputVector[0][i * cols + j] << " " << (int)inputVector[1][i * cols + j] << " " << (int)inputVector[2][i * cols + j] << "\n";
				uint32_t pixelValue1 = inputVector[i * j + j];
				image.at<uchar>(i, j) = static_cast<uchar>(pixelValue1);
			}
		}

		return image;
	}

	void filterImage(int *in, int *out, int size)
	{
		for (size_t i = 0; i < size; i++) {
			out[i] = (in[i] > 150 ? 150 : in[i]);
			// std::cout << i << "\n\tin" << (int)in[i] << "\n\tout: " << out[i] << std::endl;
		}
	}

	std::vector<int> flatten(cv::Mat image)
	{
		std::vector<int> out;
		out.reserve(image.rows*image.cols);

		std::cout << "H: " << image.rows << " W: " << image.cols << std::endl;
		for (size_t r = 0; r < image.rows; r++) {
			for (size_t c = 0; c < image.cols; c++) {
				auto tmp = image.at<cv::Vec3b>(r, c);
				out.push_back(tmp[0]);
			}
		}

		std::cout << "Returning flattened vector\n";
		return out;
	}

	void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		// convert ros Image to cv Image.
		std::cout << "encoding: " << msg->encoding << std::endl;
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
		cv::Mat img = cv_ptr->image;

		// Convert the RGB picture to Grayscale picture.
		cv::Mat grayscaleImage;
		cv::cvtColor(img, grayscaleImage, cv::COLOR_RGB2GRAY);

		// Resize the image from 640x420 to 16x16.
		cv::Mat img_small;
		resize(grayscaleImage, img_small, cv::Size(16, 16), cv::INTER_LINEAR);

		std::cout << "H: " << img_small.rows << " W: " << img_small.cols << std::endl;

		// Reshape data from 2D representation to 1D vector.
		std::vector<int> flattenedStdVector = flatten(img_small);
		std::cout << "Flattened size: " << flattenedStdVector.size() << std::endl;

		int *data = flattenedStdVector.data();
		// filterImage(data, data, 16*16);

		// filterImage(data1, data1, DATA_SIZE);
		// Write data to the HLS generated IP.
		XImge_processor_Write_in_r_Words(&ip_inst, 0, data, DATA_SIZE);
		std::cout << "data1 written to the fpga" << std::endl;
		// Start the execution of the IP block.
		XImge_processor_Start(&ip_inst);
		// Wait until the data are processed.
		while (!XImge_processor_IsDone(&ip_inst));
		// Read the output from the IP.
		std::cout << "filtered" << std::endl;
		XImge_processor_Read_out_r_Words(&ip_inst, 0, data, DATA_SIZE);
		std::cout << "data read" << std::endl;

		// Convert the read data to 16x16 cv::Mat Image using the function vectorToMat.
		cv::Mat filteredImg(16, 16, CV_8UC1);
		for (int row = 0; row < 16; ++row) {
			for (int col = 0; col < 16; ++col) {
				filteredImg.at<uchar>(row, col) = flattenedStdVector[row * col + col];
			}
		}		// auto filteredImg = vectorToMat(flattenedStdVector, 16, 16);
		std::cout << "Back to matrix" << std::endl;

		// Create ros Image from cv Image
		auto out = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", filteredImg).toImageMsg();
		std::cout << "imagge to be sent" << std::endl;

		// Publish the processed image
		publisher_->publish(*out);
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

