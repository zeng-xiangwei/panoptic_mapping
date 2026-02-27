#ifndef PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_
#define PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/common/globals.h>
#include <panoptic_mapping/integration/tsdf_integrator_base.h>
#include <panoptic_mapping/map/submap.h>
#include <panoptic_mapping/map/submap_collection.h>
#include <panoptic_mapping/map_management/map_manager_base.h>
#include <panoptic_mapping/tools/data_writer_base.h>
#include <panoptic_mapping/tools/planning_interface.h>
#include <panoptic_mapping/tools/thread_safe_submap_collection.h>
#include <panoptic_mapping/tracking/id_tracker_base.h>
#include <panoptic_mapping_msgs/srv/remove_submap.hpp>
#include <panoptic_mapping_msgs/srv/save_load_map.hpp>
#include <panoptic_mapping_msgs/srv/set_visualization_mode.hpp>
#include <panoptic_mapping_msgs/srv/submap_class_name_change.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "panoptic_mapping/integration/single_tsdf_for_undetected.h"
#include "panoptic_mapping/tools/image_data_manager.h"
#include "panoptic_mapping_msgs/msg/b_box.hpp"
#include "panoptic_mapping_msgs/msg/get_object_info_mode.hpp"
#include "panoptic_mapping_msgs/srv/get_object_info.hpp"
#include "panoptic_mapping_msgs/srv/get_submap_image_data.hpp"
#include "panoptic_mapping_ros/input/input_synchronizer.h"
#include "panoptic_mapping_ros/visualization/changed_submap_visualizer.h"
#include "panoptic_mapping_ros/visualization/planning_visualizer.h"
#include "panoptic_mapping_ros/visualization/submap_visualizer.h"
#include "panoptic_mapping_ros/visualization/tracking_visualizer.h"

#ifdef VLN_MSGS_FOUND
#include "vln_msgs/msg/b_box.hpp"
#include "vln_msgs/msg/get_object_info_mode.hpp"
#include "vln_msgs/srv/get_object_info.hpp"
#include "vln_msgs/srv/get_submap_image_data.hpp"
#endif

namespace panoptic_mapping {

class PanopticMapper {
#ifdef VLN_MSGS_FOUND
  using GetSubmapImageData = vln_msgs::srv::GetSubmapImageData;
  using VLLMProcessing = vln_msgs::srv::GetObjectInfo;
  using VLLMProcessingMode = vln_msgs::msg::GetObjectInfoMode;
  using VLLMProcessingBBox = vln_msgs::msg::BBox;
#else
  using GetSubmapImageData = panoptic_mapping_msgs::srv::GetSubmapImageData;
  using VLLMProcessing = panoptic_mapping_msgs::srv::GetObjectInfo;
  using VLLMProcessingMode = panoptic_mapping_msgs::msg::GetObjectInfoMode;
  using VLLMProcessingBBox = panoptic_mapping_msgs::msg::BBox;
#endif

 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 2;

    // Frame name used for the global frame (often mission, world, or odom).
    std::string global_frame_name = "mission";

    // How frequently to perform tasks. Execution period in seconds. Use -1 for
    // every frame, 0 for never.
    float visualization_interval = -1.f;
    float data_logging_interval = 0.f;
    float print_timing_interval = 0.f;

    // If true maintain and update the threadsafe submap collection for access.
    bool use_threadsafe_submap_collection = false;

    // Number of threads used for ROS spinning.
    int ros_spinner_threads = std::thread::hardware_concurrency();

    // Frequency in seconds in which the input queue is queried.
    float check_input_interval = 0.01f;

    // If true loaded submaps change states are set to unknown, otherwise to
    // persistent.
    bool load_submaps_conservative = true;

    // If true, keeps the active freespace ID as set in the loaded map
    bool loaded_freespace_stays_active = false;

    // If true, finish mapping and shutdown the panoptic mapper when no frames
    // are received for 3 seconds after the first frame was received.
    bool shutdown_when_finished = false;

    // Set this string to automatically save the map to the specified file when
    // shutting down when finished.
    std::string save_map_path_when_finished = "";

    // If true, display units when printing the component configs.
    bool display_config_units = true;

    // If true, indicate the default values when printing component configs.
    bool indicate_default_values = true;

    // 是否使用文件保存的embedding向量，如果保存的地图文件与当前使用的检测分割模型不一致，则该变量应该置为false
    bool use_saved_embeddings = true;

    // 是否使用图像管理功能，包括 VLLM 的调用、图像管理
    bool use_image_data_manager = true;
    // 调用 VL 大模型服务的超时时间，单位秒
    float vllm_service_timeout = 60.0;
    // 调用 VL 大模型服务失败的重试次数
    int vllm_max_retries = 0;

    // 是否启用未检测区域的TSDF重建
    bool use_undetected_tsdf = false;

    Config() { setConfigName("PanopticMapper"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Construction.
  PanopticMapper(rclcpp::Node::SharedPtr node);
  virtual ~PanopticMapper();

  // ROS callbacks.
  // Timers.
  void publishVisualizationCallback();
  void dataLoggingCallback();
  void printTimingsCallback();
  void inputCallback();

  // Services.
  bool saveMapCallback(
      const panoptic_mapping_msgs::srv::SaveLoadMap::Request::SharedPtr
          request,  // NOLINT
      panoptic_mapping_msgs::srv::SaveLoadMap::Response::SharedPtr
          response);  // NOLINT
  bool loadMapCallback(
      const panoptic_mapping_msgs::srv::SaveLoadMap::Request::SharedPtr
          request,  // NOLINT
      panoptic_mapping_msgs::srv::SaveLoadMap::Response::SharedPtr
          response);  // NOLINT
  bool setVisualizationModeCallback(
      const panoptic_mapping_msgs::srv::SetVisualizationMode::Request::SharedPtr
          request,  // NOLINT
      panoptic_mapping_msgs::srv::SetVisualizationMode::Response::SharedPtr
          response);  // NOLINT
  bool printTimingsCallback(
      const std_srvs::srv::Empty::Request::SharedPtr request,  // NOLINT
      std_srvs::srv::Empty::Response::SharedPtr response);     // NOLINT
  bool finishMappingCallback(
      const std_srvs::srv::Empty::Request::SharedPtr request,  // NOLINT
      std_srvs::srv::Empty::Response::SharedPtr response);     // NOLINT

  bool runningSwitchCallback(
      const std_srvs::srv::Empty::Request::SharedPtr request,  // NOLINT
      std_srvs::srv::Empty::Response::SharedPtr response);     // NOLINT

  bool removeSubmapCallback(
      const panoptic_mapping_msgs::srv::RemoveSubmap::Request::SharedPtr
          request,
      panoptic_mapping_msgs::srv::RemoveSubmap::Response::SharedPtr response);

  bool changeSubmapClassNameCallback(
      const panoptic_mapping_msgs::srv::SubmapClassNameChange::Request::
          SharedPtr request,
      panoptic_mapping_msgs::srv::SubmapClassNameChange::Response::SharedPtr
          response);

  bool getSubmapImageDataCallback(
      const GetSubmapImageData::Request::SharedPtr request,
      GetSubmapImageData::Response::SharedPtr response);

  // Processing.
  // Integrate a set of input images. The input is usually gathered from ROS
  // topics and provided by the InputSynchronizer.
  void processInput(InputData* input);

  // Performs various post-processing actions.
  // NOTE(schmluk): This is currently a preliminary tool to play around with.
  void finishMapping();

  // IO.
  bool saveMap(const std::string& file_path);
  bool loadMap(const std::string& file_path);

  // Utilities.
  // Print all timings (from voxblox::timing) to console.
  void printTimings() const;

  // Update the meshes and publish the all visualizations of the current map.
  void publishVisualization();

  // Pulish instance segmented point cloud from depth image and id_image
  void publishSegmentedPointCloud(InputData* input);
  // Publish colored point cloud from depth image and color image
  void publishColoredPointCloud(InputData* input);

  // Access.
  const SubmapCollection& getSubmapCollection() const { return *submaps_; }
  const ThreadSafeSubmapCollection& getThreadSafeSubmapCollection() const {
    return *thread_safe_submaps_;
  }
  const PlanningInterface& getPlanningInterface() const {
    return *planning_interface_;
  }
  MapManagerBase* getMapManagerPtr() { return map_manager_.get(); }
  const Config& getConfig() const { return config_; }

 private:
  // Setup.
  void setupMembers();
  void setupMembersFromYaml();
  void setupCollectionDependentMembers();
  void setupRos();
  bool saveIsoSurfacePoints(const std::string& file_path);
  bool saveIsoSurfacePointsForSingleTsdf(const std::string& file_path);

  // 图像管理线程函数
  void imageManagementThread();
  void vllmProcessingResponse(VLLMProcessing::Response::SharedPtr response,
                              VLLMProcessing::Request::SharedPtr request,
                              VLLMOutputData& vllm_output);
  // 获取图像管理模块未处理的图像数据，并调用VL大模型服务进行处理
  void processNotProcessedImageDataForVLLM();
  // 组织VLLM的请求内容
  std::vector<VLLMProcessing::Request::SharedPtr> prepareVllmRequest(
      std::shared_ptr<ImageData> image_data,
      const std::unordered_set<int>& generated_vllm_submap_ids);
  bool callVLLMServiceWithRetry(VLLMProcessing::Request::SharedPtr request,
                                VLLMProcessing::Response::SharedPtr& response,
                                int max_retries = 0);

 private:
  // Node handles.
  rclcpp::Node::SharedPtr node_;

  // Subscribers, Publishers, Services, Timers.
  rclcpp::Service<panoptic_mapping_msgs::srv::SaveLoadMap>::SharedPtr
      load_map_srv_;
  rclcpp::Service<panoptic_mapping_msgs::srv::SaveLoadMap>::SharedPtr
      save_map_srv_;
  rclcpp::Service<panoptic_mapping_msgs::srv::SetVisualizationMode>::SharedPtr
      set_visualization_mode_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_color_mode_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr print_timings_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr finish_mapping_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr running_switch_srv_;
  rclcpp::Service<panoptic_mapping_msgs::srv::RemoveSubmap>::SharedPtr
      remove_submap_srv_;
  rclcpp::Service<panoptic_mapping_msgs::srv::SubmapClassNameChange>::SharedPtr
      submap_class_name_change_srv_;
  rclcpp::Service<GetSubmapImageData>::SharedPtr get_submap_image_data_srv_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::TimerBase::SharedPtr data_logging_timer_;
  rclcpp::TimerBase::SharedPtr print_timing_timer_;
  rclcpp::TimerBase::SharedPtr input_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      segmented_point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      colored_point_cloud_pub_;

  rclcpp::Client<VLLMProcessing>::SharedPtr vllm_processing_client_;

  // Members.
  YAML::Node root_yaml_;
  const Config config_;

  // Map.
  std::shared_ptr<SubmapCollection> submaps_;
  std::shared_ptr<ThreadSafeSubmapCollection> thread_safe_submaps_;

  // Mapping.
  std::unique_ptr<IDTrackerBase> id_tracker_;
  std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;
  std::unique_ptr<MapManagerBase> map_manager_;

  // Optional undetected region TSDF reconstruction.
  std::shared_ptr<SubmapCollection> undetected_submaps_;
  std::unique_ptr<TsdfIntegratorBase> undetected_tsdf_integrator_;

  // Tools.
  std::shared_ptr<Globals> globals_;
  std::unique_ptr<InputSynchronizer> input_synchronizer_;
  std::unique_ptr<DataWriterBase> data_logger_;
  std::shared_ptr<PlanningInterface> planning_interface_;

  // Visualization.
  std::unique_ptr<SubmapVisualizer> submap_visualizer_;
  std::unique_ptr<PlanningVisualizer> planning_visualizer_;
  std::unique_ptr<TrackingVisualizer> tracking_visualizer_;
  std::unique_ptr<ChangedSubmapVisualizer> changed_submap_visualizer_;
  std::unique_ptr<SubmapVisualizer> single_tsdf_submap_visualizer_;

  // Which processing to perform.
  bool compute_vertex_map_ = false;
  bool compute_validity_image_ = false;

  // Tracking variables.
  std::chrono::system_clock::time_point previous_frame_time_ =
      std::chrono::system_clock::now();
  std::unique_ptr<Timer> frame_timer_;
  rclcpp::Time last_input_;
  bool got_a_frame_ = false;

  // Default namespaces and types for modules are defined here.
  static const std::map<std::string, std::pair<std::string, std::string>>
      default_names_and_types_;

  // Yaml
  std::string defaultYamlKeyPath(const std::string& key);
  YAML::Node loadYaml();

  // Mainly used for ros services
  std::mutex node_mutex_;

  // Run or stop, controled by service
  bool stop_running_ = false;
  rclcpp::CallbackGroup::SharedPtr submap_service_callback_group_;

  // 图像管理相关成员
  std::unique_ptr<ImageDataManager> image_data_manager_;
  std::mutex image_management_queue_mutex_;
  std::condition_variable image_management_cv_;
  std::thread image_management_thread_;
  bool image_management_thread_running_ = true;
  rclcpp::CallbackGroup::SharedPtr image_service_callback_group_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_PANOPTIC_MAPPER_H_
