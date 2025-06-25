#ifndef PANOPTIC_MAPPING_ROS_VISUALIZATION_TRACKING_VISUALIZER_H_
#define PANOPTIC_MAPPING_ROS_VISUALIZATION_TRACKING_VISUALIZER_H_

#include <string>
#include <unordered_map>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/tracking/id_tracker_base.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace panoptic_mapping {

class TrackingVisualizer {
 public:
  // config
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
    bool visualize_tracking = true;
    std::string ros_namespace;

    Config() { setConfigName("TrackingVisualizer"); }

   protected:
    void setupParamsAndPrinting() override;
    void fromRosParam() override;
  };

  // Constructors.
  explicit TrackingVisualizer(const Config& config,
                              rclcpp::Node::SharedPtr node);
  virtual ~TrackingVisualizer() = default;

  // Setup.
  void registerIDTracker(IDTrackerBase* tracker);

  // Publish visualization requests.
  void publishImage(const cv::Mat& image, const std::string& name);

 private:
  const Config config_;

  // Publishers.
  rclcpp::Node::SharedPtr node_;
  std::unordered_map<std::string,
                     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
      publishers_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_VISUALIZATION_TRACKING_VISUALIZER_H_
