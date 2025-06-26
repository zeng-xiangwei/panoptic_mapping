#include "panoptic_mapping_ros/visualization/tracking_visualizer.h"

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

namespace panoptic_mapping {

void TrackingVisualizer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("visualize_tracking", &visualize_tracking);
}

void TrackingVisualizer::Config::fromRosParam() {
  ros_namespace = rosParamNameSpace();
}

TrackingVisualizer::TrackingVisualizer(const Config& config,
                                       rclcpp::Node::SharedPtr node)
    : config_(config.checkValid()), node_(node) {
  // Print config after setting up the modes.
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void TrackingVisualizer::registerIDTracker(IDTrackerBase* tracker) {
  if (config_.visualize_tracking) {
    CHECK_NOTNULL(tracker);
    tracker->setVisualizationCallback(
        [this](const cv::Mat& image, const std::string& name) {
          publishImage(image, name);
        });
  }
}

void TrackingVisualizer::publishImage(const cv::Mat& image,
                                      const std::string& name) {
  auto it = publishers_.find(name);
  if (it == publishers_.end()) {
    // Advertise a new topic if there is no publisher for the given name.
    it = publishers_
             .emplace(name, node_->create_publisher<sensor_msgs::msg::Image>(
                                name, 100))
             .first;
  }

  // Publish the image, expected as BGR8.
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Clock().now();
  auto image_msg_ptr =
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image)
          .toImageMsg();
  it->second->publish(*image_msg_ptr);
}

}  // namespace panoptic_mapping
