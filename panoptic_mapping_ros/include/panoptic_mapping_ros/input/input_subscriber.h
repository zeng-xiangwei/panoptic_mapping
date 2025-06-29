#ifndef PANOPTIC_MAPPING_ROS_INPUT_INPUT_SUBSCRIBER_H_
#define PANOPTIC_MAPPING_ROS_INPUT_INPUT_SUBSCRIBER_H_

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <panoptic_mapping/common/input_data.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "panoptic_mapping_ros/input/input_synchronizer.h"

namespace panoptic_mapping {

/**
 * @brief Wrapper for input data that is used to store additional tracking info.
 */
struct InputSynchronizerData {
  std::shared_ptr<InputData> data;
  bool valid = true;
  bool ready = false;
  rclcpp::Time timestamp;
  std::mutex write_mutex_;  // Lock this mutex when writing to common data
                            // structures such as the input list
};

/**
 * @brief Define a base interface to input synchronizers.
 */
class InputSynchronizerBase {
 public:
  InputSynchronizerBase() = default;
  virtual ~InputSynchronizerBase() = default;
  virtual bool getDataInQueue(const rclcpp::Time& timestamp,
                              InputSynchronizerData** data) = 0;
  virtual void checkDataIsReady(InputSynchronizerData* data) = 0;
};

// Some template specializations to lookup the timestamp of a message.
template <typename MsgT>
inline rclcpp::Time getTimeStampFromMsg(const MsgT& msg) {
  return msg.header.stamp;
}

inline rclcpp::Time getTimeStampFromMsg(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  return msg->header.stamp;
}

/**
 * @brief Tool to manage the subscription and and extraction of each input
 * topic.
 */
class InputSubscriberBase {
 public:
  InputSubscriberBase() = default;
  virtual ~InputSubscriberBase() = default;
};

/**
 * @brief Implementation of input subscribers for different messages as tool to
 * manage the subscription and and extraction of each input topic
 *
 * @tparam MsgT Type of the ROS-message to subscribe to.
 */
template <typename MsgT>
class InputSubscriber : public InputSubscriberBase {
 public:
  InputSubscriber(rclcpp::Node::SharedPtr node, const std::string& topic_name,
                  int queue_size,
                  std::function<void(const typename MsgT::SharedPtr,
                                     InputSynchronizerData*)>
                      extraction_function,
                  InputSynchronizerBase* parent)
      : extraction_function_(std::move(extraction_function)),
        node_(node),
        parent_(parent) {
    // Subscribe to the topic.
    subscriber_ = node_->create_subscription<MsgT>(
        topic_name, queue_size,
        [this](const typename MsgT::SharedPtr msg) { this->msgCallback(msg); });
  }

  void msgCallback(const typename MsgT::SharedPtr msg) {
    // Store the input message data in the queue.
    const rclcpp::Time stamp = getTimeStampFromMsg(*msg);
    InputSynchronizerData* data;
    if (parent_->getDataInQueue(stamp, &data)) {
      extraction_function_(msg, data);
      parent_->checkDataIsReady(data);
    }
  }

 private:
  // ROS Subscriber.
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Subscription<MsgT>::SharedPtr subscriber_;

  // Tells the queue how to store its data in the input data.
  std::function<void(const typename MsgT::SharedPtr msg, InputSynchronizerData* data)>
      extraction_function_;

  // Reference to the parent.
  InputSynchronizerBase* const parent_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_INPUT_INPUT_SUBSCRIBER_H_
