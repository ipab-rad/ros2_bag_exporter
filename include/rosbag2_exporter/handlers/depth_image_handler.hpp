/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__DEPTH_IMAGE_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__DEPTH_IMAGE_HANDLER_HPP_

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace rosbag2_exporter
{

class DepthImageHandler : public BaseHandler
{
public:
  // Constructor to accept logger and encoding with default fallback
  DepthImageHandler(const std::string & topic_dir,
                   const std::string & encoding,
                   rclcpp::Logger logger)
  : BaseHandler(logger), topic_dir_(topic_dir)
  {
    // Validate or set default encoding if not provided
    if (encoding.empty()) {
      RCLCPP_WARN(logger, "No encoding provided. Defaulting to '16UC1'.");
      encoding_ = "16UC1";  // Default encoding for depth images
    } else {
      encoding_ = encoding;
    }
  }

  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                      const std::string & topic,
                      size_t index) override
  {
    // Deserialize the incoming message
    sensor_msgs::msg::Image img;
    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    serializer.deserialize_message(&serialized_msg, &img);

    // Convert the sensor message to a cv::Mat image using cv_bridge
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, encoding_);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(logger_, "CV Bridge exception: %s. Using default encoding '16UC1'.", e.what());

      // Attempt to fallback to the default '16UC1' encoding
      try {
        cv_ptr = cv_bridge::toCvCopy(img, "16UC1");
        encoding_ = "16UC1";  // Update to fallback encoding
      } catch (const cv_bridge::Exception & e2) {
        RCLCPP_ERROR(logger_, "Fallback to '16UC1' failed: %s", e2.what());
        return;
      }
    }

    // Create a timestamped filename
    std::stringstream ss_timestamp;
    ss_timestamp << img.header.stamp.sec << "-"
                << std::setw(9) << std::setfill('0') << img.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Create the full file path with '.png' as the extension
    std::string filepath = topic_dir_ + "/" + timestamp + ".png";

    // Ensure the directory exists, create if necessary
    if (!std::filesystem::exists(topic_dir_)) {
      std::filesystem::create_directories(topic_dir_);
    }

    // Normalize depth image for better visualization if needed
    cv::Mat depth_image;
    if (encoding_ == "16UC1") {
      cv_ptr->image.convertTo(depth_image, CV_16UC1);  // No normalization applied here
    } else {
      depth_image = cv_ptr->image;  // Handle other encodings
    }

    // Write the depth image to disk
    if (!cv::imwrite(filepath, depth_image)) {
      RCLCPP_ERROR(logger_, "Failed to write depth image to %s", filepath.c_str());
    } else {
        data_meta_vec_.push_back(DataMeta{filepath, img.header.stamp, index});
        RCLCPP_DEBUG(logger_, "Successfully wrote depth image to %s", filepath.c_str());
    }
  }

private:
  std::string topic_dir_;
  std::string encoding_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__DEPTH_IMAGE_HANDLER_HPP_
