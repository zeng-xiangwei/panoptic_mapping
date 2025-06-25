#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include "panoptic_mapping_utils/evaluation/map_evaluator.h"

int main(int argc, char** argv) {
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});

  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Run ros.
  rclcpp::init(argc, argv);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Evaluator.
  panoptic_mapping::MapEvaluator evaluator(nh, nh_private);
  if (evaluator.setupMultiMapEvaluation()) {
    ros::spin();
  }
  return 0;
}
