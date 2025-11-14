#include "panoptic_mapping_ros/panoptic_mapper.h"

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <panoptic_mapping/common/camera.h>
#include <panoptic_mapping/labels/label_handler_base.h>
#include <panoptic_mapping/map/classification/fixed_count.h>
#include <panoptic_mapping/submap_allocation/freespace_allocator_base.h>
#include <panoptic_mapping/submap_allocation/submap_allocator_base.h>

#include "panoptic_mapping_ros/conversions/conversions.h"

#include "cv_bridge/cv_bridge.h"

namespace panoptic_mapping {

// Modules that don't have a default type will be required to be explicitly set.
// Entries: <key, <ros_namespace, default type parameter>.
const std::map<std::string, std::pair<std::string, std::string>>
    PanopticMapper::default_names_and_types_ = {
        {"camera", {"camera", ""}},
        {"label_handler", {"labels", "null"}},
        {"submap_allocator", {"submap_allocator", "null"}},
        {"freespace_allocator", {"freespace_allocator", "null"}},
        {"id_tracker", {"id_tracker", ""}},
        {"tsdf_integrator", {"tsdf_integrator", ""}},
        {"map_management", {"map_management", "null"}},
        {"vis_submaps", {"visualization/submaps", "submaps"}},
        {"vis_tracking", {"visualization/tracking", ""}},
        {"vis_planning", {"visualization/planning", ""}},
        {"vis_changed_submaps", {"visualization/changed_submaps", ""}},
        {"data_writer", {"data_writer", "null"}},
        {"image_data_manager", {"image_data_manager", ""}}};

void PanopticMapper::Config::checkParams() const {
  checkParamCond(!global_frame_name.empty(),
                 "'global_frame_name' may not be empty.");
  checkParamGT(ros_spinner_threads, 1, "ros_spinner_threads");
  checkParamGT(check_input_interval, 0.f, "check_input_interval");
}

void PanopticMapper::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("global_frame_name", &global_frame_name);
  setupParam("visualization_interval", &visualization_interval, "s");
  setupParam("data_logging_interval", &data_logging_interval, "s");
  setupParam("print_timing_interval", &print_timing_interval, "s");
  setupParam("use_threadsafe_submap_collection",
             &use_threadsafe_submap_collection);
  setupParam("ros_spinner_threads", &ros_spinner_threads);
  setupParam("check_input_interval", &check_input_interval, "s");
  setupParam("load_submaps_conservative", &load_submaps_conservative);
  setupParam("loaded_freespace_stays_active", &loaded_freespace_stays_active);
  setupParam("shutdown_when_finished", &shutdown_when_finished);
  setupParam("save_map_path_when_finished", &save_map_path_when_finished);
  setupParam("display_config_units", &display_config_units);
  setupParam("indicate_default_values", &indicate_default_values);
  setupParam("use_saved_embeddings", &use_saved_embeddings);
  setupParam("vllm_service_timeout", &vllm_service_timeout);
  setupParam("use_image_data_manager", &use_image_data_manager);
  setupParam("vllm_max_retries", &vllm_max_retries);
}

PanopticMapper::PanopticMapper(rclcpp::Node::SharedPtr node)
    : node_(node),
      root_yaml_(loadYaml()),
      config_(config_utilities::getConfigFromYaml<PanopticMapper::Config>(
                  root_yaml_)
                  .checkValid()) {
  // Setup printing of configs.
  // NOTE(schmluk): These settings are global so multiple panoptic mappers in
  // the same process might interfere.
  config_utilities::GlobalSettings().indicate_default_values =
      config_.indicate_default_values;
  config_utilities::GlobalSettings().indicate_units =
      config_.display_config_units;
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // Setup all components of the panoptic mapper.
  setupMembersFromYaml();
  setupRos();
}

PanopticMapper::~PanopticMapper() {
  // 停止图像管理线程
  {
    std::lock_guard<std::mutex> lock(image_management_queue_mutex_);
    image_management_thread_running_ = false;
  }
  image_management_cv_.notify_all();

  if (image_management_thread_.joinable()) {
    image_management_thread_.join();
  }
}

void PanopticMapper::setupMembersFromYaml() {
  // Map.
  submaps_ = std::make_shared<SubmapCollection>();

  // Threadsafe wrapper for the map.
  thread_safe_submaps_ = std::make_shared<ThreadSafeSubmapCollection>(submaps_);

  // Camera.
  auto camera = std::make_shared<Camera>(
      config_utilities::getConfigFromYaml<Camera::Config>(
          root_yaml_, defaultYamlKeyPath("camera")));

  // Label Handler.
  std::shared_ptr<LabelHandlerBase> label_handler =
      config_utilities::FactoryYaml::create<LabelHandlerBase>(
          root_yaml_, defaultYamlKeyPath("label_handler"));

  // Setup the number of labels.
  FixedCountVoxel::setNumCounts(label_handler->numberOfLabels());

  // Globals.
  globals_ = std::make_shared<Globals>(camera, label_handler);

  // Submap Allocation.
  std::shared_ptr<SubmapAllocatorBase> submap_allocator =
      config_utilities::FactoryYaml::create<SubmapAllocatorBase>(
          root_yaml_, defaultYamlKeyPath("submap_allocator"));
  std::shared_ptr<FreespaceAllocatorBase> freespace_allocator =
      config_utilities::FactoryYaml::create<FreespaceAllocatorBase>(
          root_yaml_, defaultYamlKeyPath("freespace_allocator"));

  // ID Tracking.
  id_tracker_ = config_utilities::FactoryYaml::create<IDTrackerBase>(
      root_yaml_, defaultYamlKeyPath("id_tracker"), globals_);
  id_tracker_->setSubmapAllocator(submap_allocator);
  id_tracker_->setFreespaceAllocator(freespace_allocator);

  // Tsdf Integrator.
  tsdf_integrator_ = config_utilities::FactoryYaml::create<TsdfIntegratorBase>(
      root_yaml_, defaultYamlKeyPath("tsdf_integrator"), globals_);

  // Map Manager.
  map_manager_ = config_utilities::FactoryYaml::create<MapManagerBase>(
      root_yaml_, defaultYamlKeyPath("map_management"), globals_);

  // Submaps.
  submap_visualizer_ = config_utilities::FactoryYaml::create<SubmapVisualizer>(
      root_yaml_, defaultYamlKeyPath("vis_submaps"), globals_, node_);
  submap_visualizer_->setGlobalFrameName(config_.global_frame_name);

  // Tracking.
  tracking_visualizer_ = std::make_unique<TrackingVisualizer>(
      config_utilities::getConfigFromYaml<TrackingVisualizer::Config>(
          root_yaml_, defaultYamlKeyPath("vis_tracking")),
      node_);
  tracking_visualizer_->registerIDTracker(id_tracker_.get());

  // For demo, changed submap visualizer.
  changed_submap_visualizer_ = std::make_unique<ChangedSubmapVisualizer>(
      config_utilities::getConfigFromYaml<ChangedSubmapVisualizer::Config>(
          root_yaml_, defaultYamlKeyPath("vis_changed_submaps")),
      node_);

  // Planning.
  setupCollectionDependentMembers();

  // Data Logging.
  data_logger_ = config_utilities::FactoryYaml::create<DataWriterBase>(
      root_yaml_, defaultYamlKeyPath("data_writer"));

  // 图像管理器
  if (config_.use_image_data_manager) {
    image_data_manager_ = std::make_unique<ImageDataManager>(
        config_utilities::getConfigFromYaml<ImageDataManager::Config>(
            root_yaml_, defaultYamlKeyPath("image_data_manager")));
  }

  // Setup all requested inputs from all modules.
  InputData::InputTypes requested_inputs;
  std::vector<InputDataUser*> input_data_users = {
      id_tracker_.get(), tsdf_integrator_.get(), submap_allocator.get(),
      freespace_allocator.get()};
  for (const InputDataUser* input_data_user : input_data_users) {
    requested_inputs.insert(input_data_user->getRequiredInputs().begin(),
                            input_data_user->getRequiredInputs().end());
  }
  compute_vertex_map_ =
      requested_inputs.find(InputData::InputType::kVertexMap) !=
      requested_inputs.end();
  compute_validity_image_ =
      requested_inputs.find(InputData::InputType::kValidityImage) !=
      requested_inputs.end();

  // Setup the input synchronizer.
  input_synchronizer_ = std::make_unique<InputSynchronizer>(
      config_utilities::getConfigFromYaml<InputSynchronizer::Config>(
          root_yaml_),
      node_);
  input_synchronizer_->requestInputs(requested_inputs);
}

void PanopticMapper::setupCollectionDependentMembers() {
  // Planning Interface.
  planning_interface_ = std::make_shared<PlanningInterface>(submaps_);

  // Planning Visualizer.
  planning_visualizer_ = std::make_unique<PlanningVisualizer>(
      config_utilities::getConfigFromYaml<PlanningVisualizer::Config>(
          root_yaml_, defaultYamlKeyPath("vis_planning")),
      planning_interface_, node_);
  planning_visualizer_->setGlobalFrameName(config_.global_frame_name);
}

void PanopticMapper::setupRos() {
  bool load_map;
  node_->get_parameter("load_map", load_map);
  if (load_map) {
    std::string load_file;
    node_->get_parameter("load_file", load_file);
    if (!loadMap(load_file)) {
      CHECK(false) << "Failed to load map from " << load_file;
    }

    if (image_data_manager_) {
      image_data_manager_->loadMap();
    }
  }

  // 创建独立的回调组用于图像服务
  image_service_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  submap_service_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Setup all input topics.
  input_synchronizer_->advertiseInputTopics();

  // Services.
  save_map_srv_ =
      node_->create_service<panoptic_mapping_msgs::srv::SaveLoadMap>(
          "save_map",
          std::bind(&PanopticMapper::saveMapCallback, this,
                    std::placeholders::_1, std::placeholders::_2),
          rmw_qos_profile_services_default, submap_service_callback_group_);

  set_visualization_mode_srv_ =
      node_->create_service<panoptic_mapping_msgs::srv::SetVisualizationMode>(
          "set_visualization_mode",
          std::bind(&PanopticMapper::setVisualizationModeCallback, this,
                    std::placeholders::_1, std::placeholders::_2),
          rmw_qos_profile_services_default, submap_service_callback_group_);
  print_timings_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "print_timings",
      [this](const std_srvs::srv::Empty::Request::SharedPtr req,
             std_srvs::srv::Empty::Response::SharedPtr res) -> bool {
        return printTimingsCallback(req, res);
      },
      rmw_qos_profile_services_default, submap_service_callback_group_);
  finish_mapping_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "finish_mapping",
      std::bind(&PanopticMapper::finishMappingCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, submap_service_callback_group_);

  running_switch_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "running_switch",
      std::bind(&PanopticMapper::runningSwitchCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, submap_service_callback_group_);

  remove_submap_srv_ =
      node_->create_service<panoptic_mapping_msgs::srv::RemoveSubmap>(
          "remove_submap",
          std::bind(&PanopticMapper::removeSubmapCallback, this,
                    std::placeholders::_1, std::placeholders::_2),
          rmw_qos_profile_services_default, submap_service_callback_group_);
  submap_class_name_change_srv_ =
      node_->create_service<panoptic_mapping_msgs::srv::SubmapClassNameChange>(
          "set_submap_class_name",
          std::bind(&PanopticMapper::changeSubmapClassNameCallback, this,
                    std::placeholders::_1, std::placeholders::_2),
          rmw_qos_profile_services_default, submap_service_callback_group_);

  // 新增的图像管理服务（作为服务端）
  get_submap_image_data_srv_ = node_->create_service<GetSubmapImageData>(
      "get_submap_image_data",
      std::bind(&PanopticMapper::getSubmapImageDataCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, image_service_callback_group_);

  // VL大模型客户端
  vllm_processing_client_ = node_->create_client<VLLMProcessing>(
      "request_vl_processing", rmw_qos_profile_services_default,
      image_service_callback_group_);

  // Publishers.
  segmented_point_cloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "segmented_point_cloud", 1);
  colored_point_cloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "colored_point_cloud", 1);

  // Timers.
  if (config_.visualization_interval > 0.0) {
    visualization_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(config_.visualization_interval),
        [this]() { publishVisualizationCallback(); });
  }
  if (config_.data_logging_interval > 0.0) {
    data_logging_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(config_.data_logging_interval),
        [this]() { dataLoggingCallback(); });
  }
  if (config_.print_timing_interval > 0.0) {
    print_timing_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(config_.print_timing_interval),
        [this]() { dataLoggingCallback(); });
  }
  input_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(config_.check_input_interval),
      [this]() { inputCallback(); });

  // 启动图像管理线程
  if (image_data_manager_) {
    image_management_thread_ =
        std::thread(&PanopticMapper::imageManagementThread, this);
  }
}

void PanopticMapper::inputCallback() {
  std::lock_guard<std::mutex> lock(node_mutex_);
  if (input_synchronizer_->hasInputData()) {
    std::shared_ptr<InputData> data = input_synchronizer_->getInputData();
    if (!stop_running_ && data) {
      processInput(data.get());
      if (config_.shutdown_when_finished) {
        last_input_ = node_->get_clock()->now();
        got_a_frame_ = true;
      }
      // 保存输入数据
      if (image_data_manager_) {
        std::chrono::system_clock::time_point t0 =
            std::chrono::system_clock::now();
        Timer timer("addImageData");
        image_data_manager_->addImageData(data->colorImage(), data->idImage(),
                                          data->detectronLabels(),
                                          data->timestamp(), *submaps_);
        timer.Stop();
        std::chrono::system_clock::time_point t1 =
            std::chrono::system_clock::now();
        LOG(INFO) << "Adding one image data took "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(t1 -
                                                                           t0)
                         .count();
      }
      image_management_cv_.notify_one();
    }
  } else {
    if (config_.shutdown_when_finished && got_a_frame_ &&
        (node_->get_clock()->now() - last_input_).seconds() >= 3.0) {
      // No more frames, finish up.
      LOG_IF(INFO, config_.verbosity >= 1)
          << "No more frames received for 3 seconds, shutting down.";
      finishMapping();
      if (!config_.save_map_path_when_finished.empty()) {
        saveMap(config_.save_map_path_when_finished);
      }
      LOG_IF(INFO, config_.verbosity >= 1) << "Finished.";
      rclcpp::shutdown();
    }
  }
}

void PanopticMapper::processInput(InputData* input) {
  CHECK_NOTNULL(input);
  Timer timer("input");
  frame_timer_ = std::make_unique<Timer>("frame");

  // Compute and store the validity image.
  if (compute_validity_image_) {
    Timer validity_timer("input/compute_validity_image");
    input->setValidityImage(
        globals_->camera()->computeValidityImage(input->depthImage()));
  }

  // Compute and store the vertex map.
  if (compute_vertex_map_) {
    Timer vertex_timer("input/compute_vertex_map");
    input->setVertexMap(
        globals_->camera()->computeVertexMap(input->depthImage()));
  }
  std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();

  // Track the segmentation images and allocate new submaps.
  Timer id_timer("input/id_tracking");
  id_tracker_->processInput(submaps_.get(), input);
  std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
  id_timer.Stop();

  // Integrate the images.
  Timer tsdf_timer("input/tsdf_integration");
  tsdf_integrator_->processInput(submaps_.get(), input);
  std::chrono::system_clock::time_point t2 = std::chrono::system_clock::now();
  tsdf_timer.Stop();

  // Perform all requested map management actions.
  Timer management_timer("input/map_management");
  map_manager_->tick(submaps_.get(), input);
  std::chrono::system_clock::time_point t3 = std::chrono::system_clock::now();
  management_timer.Stop();

  // If requested perform visualization and logging.
  if (config_.visualization_interval < 0.f) {
    Timer vis_timer("input/visualization");
    publishVisualizationCallback();
    if (segmented_point_cloud_pub_->get_subscription_count() > 0) {
      publishSegmentedPointCloud(input);
    }
    if (colored_point_cloud_pub_->get_subscription_count() > 0) {
      publishColoredPointCloud(input);
    }
  }
  if (config_.data_logging_interval < 0.f) {
    dataLoggingCallback();
  }
  std::chrono::system_clock::time_point t4 = std::chrono::system_clock::now();

  // If requested update the thread_safe_submaps.
  if (config_.use_threadsafe_submap_collection) {
    thread_safe_submaps_->update();
  }

  // Logging.
  timer.Stop();
  std::stringstream info;
  info << "Processed input data.";
  if (config_.verbosity >= 3) {
    info << "\n(tracking: "
         << static_cast<double>(
                std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                    .count())
         << " + integration: "
         << static_cast<double>(
                std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                    .count())
         << " + management: "
         << static_cast<double>(
                std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                    .count());
    if (config_.visualization_interval <= 0.f) {
      info << " + visual: "
           << static_cast<double>(
                  std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3)
                      .count());
    }
    info << " = "
         << static_cast<double>(
                std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0)
                    .count())
         << ", frame: "
         << static_cast<double>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now() - previous_frame_time_)
                    .count())
         << "ms)";
  }
  previous_frame_time_ = std::chrono::system_clock::now();
  LOG_IF(INFO, config_.verbosity >= 2) << info.str();
  LOG_IF(INFO, config_.print_timing_interval < 0.0) << "\n" << Timing::Print();
}

// 图像管理线程函数
void PanopticMapper::imageManagementThread() {
  while (image_management_thread_running_) {
    {
      std::unique_lock<std::mutex> lock(image_management_queue_mutex_);
      image_management_cv_.wait(lock, [this] {
        // vl 大模型服务可用且有未处理图像数据  或者  线程被要求停止
        return (vllm_processing_client_->service_is_ready() &&
                image_data_manager_->unprocessedImageDataSize() > 0) ||
               !image_management_thread_running_;
      });
    }

    if (!image_management_thread_running_) {
      break;
    }

    std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();

    // 将未处理的图像调用 VL 大模型
    processNotProcessedImageDataForVLLM();

    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
    LOG(INFO) << "Image management thread: one cycle took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                     .count()
              << " ms";
  }
}

void PanopticMapper::processNotProcessedImageDataForVLLM() {
  if (!vllm_processing_client_ ||
      !vllm_processing_client_->service_is_ready()) {
    LOG(WARNING) << "VLLM processing service not available.";
    return;
  }
  // 获取需要处理的图像数据
  LOG(INFO) << "unprocessed image data size: "
            << image_data_manager_->unprocessedImageDataSize()
            << ", start processing";
  while (image_data_manager_->unprocessedImageDataSize() > 0) {
    auto image_data =
        image_data_manager_->getFirstNotProcessedImageDataForVLLM();
    if (image_data) {
      // 定义回调函数处理VL大模型的响应
      LOG(INFO) << "Processing image " << image_data->image_id
                << ", call async service";

      std::chrono::system_clock::time_point t_service_0 =
          std::chrono::system_clock::now();
      // // 异步调用VL大模型服务，并等待结果，达到与同步一样的效果
      auto request = prepareVllmRequest(image_data);
      VLLMProcessing::Response::SharedPtr response;
      bool success = callVLLMServiceWithRetry(request, response, config_.vllm_max_retries);
      if (success) {
        vllmProcessingResponse(response);
      }
      // TODO: 服务调用失败是是否需要记录

      std::chrono::system_clock::time_point t_service_1 =
          std::chrono::system_clock::now();
      LOG(INFO) << "VLLM processing service finished. cost "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                       t_service_1 - t_service_0)
                       .count()
                << " ms";

      LOG(INFO) << "unprocessed image data size remaining: "
                << image_data_manager_->unprocessedImageDataSize();
    }
  }
}

// 添加重试机制的服务调用函数
bool PanopticMapper::callVLLMServiceWithRetry(
    VLLMProcessing::Request::SharedPtr request,
    VLLMProcessing::Response::SharedPtr& response, int max_retries) {
  for (int attempt = 0; attempt <= max_retries; ++attempt) {
    if (!vllm_processing_client_ ||
        !vllm_processing_client_->service_is_ready()) {
      LOG(WARNING) << "VLLM processing service not available";
      if (attempt < max_retries) {
        LOG(WARNING) << "Waiting 500 ms and retrying...";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
      return false;
    }

    auto result_future = vllm_processing_client_->async_send_request(request);
    auto status = result_future.wait_for(
        std::chrono::duration<double>(config_.vllm_service_timeout));

    if (status == std::future_status::ready) {
      try {
        response = result_future.get();
        return true;
      } catch (const std::exception& e) {
        LOG(WARNING) << "Exception when getting VLLM service response: "
                     << e.what();
      }
    } else {
      LOG(WARNING) << "VLLM service timeout on attempt " << (attempt + 1);
    }

    if (attempt < max_retries) {
      LOG(INFO) << "Retrying VLLM service call (attempt " << (attempt + 2)
                << "), waiting 500ms...";
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  LOG(ERROR) << "Failed to call VLLM service after " << (max_retries + 1)
             << " attempts";
  return false;
}

PanopticMapper::VLLMProcessing::Request::SharedPtr
PanopticMapper::prepareVllmRequest(std::shared_ptr<ImageData> image_data) {
  auto request = std::make_shared<VLLMProcessing::Request>();
  request->image.image_id = image_data->image_id;

  cv_bridge::CvImage cv_image;
  cv_image.header.stamp =
      rclcpp::Time(static_cast<int64_t>(image_data->timestamp * 1e9));
  cv_image.header.frame_id = "image";
  cv_image.encoding = "bgr8";
  cv_image.image = image_data->rgb_data;
  request->image.image = *cv_image.toImageMsg();

  request->image.mode = VLLMProcessingMode::GENERATE_MODE;
  for (const auto& [submap_id, submap_name] : image_data->associated_submaps) {
    request->image.object_names.push_back(submap_name);
  }
  return request;
}

// VL大模型服务响应的回调函数
void PanopticMapper::vllmProcessingResponse(
    VLLMProcessing::Response::SharedPtr response) {
  // 获取响应结果
  std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();

  // 检查服务调用是否成功
  if (!response->success) {
    LOG(WARNING) << "VL processing service call failed: ";
    return;
  }

  // 将ROS服务响应数据转换为内部数据格式
  VLLMOutputData vllm_output;
  vllm_output.image_id = response->image_id;

  // 转换边界框信息
  for (const auto& bbox_msg : response->objects) {
    BoundingBoxInfoByVLLM bbox_info;
    bbox_info.id = bbox_msg.id;
    bbox_info.description.class_name = bbox_msg.object_name;
    int xmin = bbox_msg.bbox[0], ymin = bbox_msg.bbox[1];
    int xmax = bbox_msg.bbox[2], ymax = bbox_msg.bbox[3];
    int width = xmax - xmin;
    int height = ymax - ymin;
    bbox_info.bounding_box = cv::Rect(xmin, ymin, width, height);
    bbox_info.description.color = bbox_msg.color;
    bbox_info.description.shape = bbox_msg.shape;
    vllm_output.bounding_boxes_info.push_back(bbox_info);
  }

  for (const auto& bbox_relation_msg : response->relationships) {
    VllmRelationship bbox_relation;
    bbox_relation.from_id = bbox_relation_msg.from_id;
    bbox_relation.to_id = bbox_relation_msg.to_id;
    bbox_relation.relationship = stringToRelationshipType(bbox_relation_msg.type);
    vllm_output.bounding_boxes_relationships.push_back(bbox_relation);
  }

  // 调用图像管理模块处理函数
  {
    std::lock_guard<std::mutex> lock(node_mutex_);
    image_data_manager_->processVLLMOutput(vllm_output, *submaps_);
  }

  std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
  LOG(INFO)
      << "Processing VLLM output took "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
      << " ms.";
}

// 获取submap对应的图像数据服务回调（作为服务端）
bool PanopticMapper::getSubmapImageDataCallback(
    const GetSubmapImageData::Request::SharedPtr request,
    GetSubmapImageData::Response::SharedPtr response) {
  // 获取与submap关联的图像
  if (!image_data_manager_) {
    LOG(ERROR) << "No image data manager set!";
    response->success = false;
    return true;
  }

  auto image_data = image_data_manager_->getImageForSubmap(request->submap_id);
  if (!image_data) {
    response->success = false;
    return true;
  }

  response->success = true;

  cv_bridge::CvImage cv_image;
  cv_image.header.stamp =
      rclcpp::Time(static_cast<int64_t>(image_data->timestamp * 1e9));
  cv_image.header.frame_id = "image";
  cv_image.encoding = "bgr8";
  cv_image.image = image_data->rgb_data;

  response->rgb_image = *cv_image.toImageMsg();

  response->img_timestamp = image_data->timestamp;
  return true;
}

void PanopticMapper::finishMapping() {
  map_manager_->finishMapping(submaps_.get());
  submap_visualizer_->visualizeAll(submaps_.get());
  LOG_IF(INFO, config_.verbosity >= 2) << "Finished mapping.";
}

void PanopticMapper::publishVisualization() {
  Timer timer("visualization");
  submap_visualizer_->visualizeAll(submaps_.get());
  planning_visualizer_->visualizeAll();
  changed_submap_visualizer_->visualizeChangedSubmaps(submaps_.get());
}

void PanopticMapper::publishSegmentedPointCloud(InputData* input) {
  Pointcloud segmented_pointcloud;
  using ColorType = Eigen::Vector<uint8_t, 3>;
  std::vector<ColorType, Eigen::aligned_allocator<ColorType>> colors;

  // 获取相机内参
  const Camera& camera = *globals_->camera();
  float fx = camera.getConfig().fx;
  float fy = camera.getConfig().fy;
  float cx = camera.getConfig().vx;
  float cy = camera.getConfig().vy;

  // 获取深度图像数据
  const cv::Mat& depth_image =
      input->depthImage();  // 假设输入的深度图为 CV_32FC1 类型
  const cv::Mat& id_image = input->idImage();  // 分割 ID 图像
  const cv::Mat& validity_image = input->validityImage();

  // 遍历深度图并生成点云
  std::unordered_map<int32_t, int> instance_id_counter;
  for (int v = 0; v < depth_image.rows; ++v) {
    for (int u = 0; u < depth_image.cols; ++u) {
      float depth = depth_image.at<float>(v, u);
      if (validity_image.at<uchar>(v, u) == 0) continue;  // 跳过无效深度值

      // 计算点的空间坐标
      float x = (u - cx) * depth / fx;
      float y = (v - cy) * depth / fy;
      float z = depth;

      // 添加点到点云
      segmented_pointcloud.push_back(Point(x, y, z));

      // 获取对应的分割 ID
      int32_t instance_id = id_image.at<int32_t>(v, u);
      instance_id_counter[instance_id]++;

      // 根据 instance_id 生成颜色（这里用简单的哈希方式）
      uint8_t r = (instance_id * 797) % 255;
      uint8_t g = (instance_id * 1499) % 255;
      uint8_t b = (instance_id * 2333) % 255;

      colors.emplace_back(ColorType(r, g, b));
    }
  }

  LOG(INFO) << "instance count: " << instance_id_counter.size();
  for (auto& pair : instance_id_counter) {
    LOG(INFO) << "instance_id: " << pair.first << ", count: " << pair.second;
  }

  // 发布点云消息
  sensor_msgs::msg::PointCloud2 segmented_pointcloud_msg;
  convertToPointCloud2(segmented_pointcloud, colors, segmented_pointcloud_msg);
  segmented_pointcloud_msg.header.stamp =
      rclcpp::Time(input->timestamp() * 1e9, rcl_clock_type_t::RCL_ROS_TIME);
  segmented_pointcloud_msg.header.frame_id = input->sensorFrameName();

  // 发布点云话题
  segmented_point_cloud_pub_->publish(segmented_pointcloud_msg);
}

void PanopticMapper::publishColoredPointCloud(InputData* input) {
  Pointcloud colored_pointcloud;
  using ColorType = Eigen::Vector<uint8_t, 3>;
  std::vector<ColorType, Eigen::aligned_allocator<ColorType>> colors;

  // 获取相机内参
  const Camera& camera = *globals_->camera();
  float fx = camera.getConfig().fx;
  float fy = camera.getConfig().fy;
  float cx = camera.getConfig().vx;
  float cy = camera.getConfig().vy;

  // 获取深度图像数据
  const cv::Mat& depth_image =
      input->depthImage();  // 假设输入的深度图为 CV_32FC1 类型
  const cv::Mat& color_image = input->colorImage();
  const cv::Mat& validity_image = input->validityImage();

  // 遍历深度图并生成点云
  for (int v = 0; v < depth_image.rows; ++v) {
    for (int u = 0; u < depth_image.cols; ++u) {
      float depth = depth_image.at<float>(v, u);
      if (validity_image.at<uchar>(v, u) == 0) continue;  // 跳过无效深度值

      // 计算点的空间坐标
      float x = (u - cx) * depth / fx;
      float y = (v - cy) * depth / fy;
      float z = depth;

      // 添加点到点云
      colored_pointcloud.push_back(Point(x, y, z));
      auto color_bgr = color_image.at<cv::Vec3b>(v, u);
      uint8_t r = color_bgr[2];
      uint8_t g = color_bgr[1];
      uint8_t b = color_bgr[0];

      colors.emplace_back(ColorType(r, g, b));
    }
  }

  // 发布点云消息
  sensor_msgs::msg::PointCloud2 colored_pointcloud_msg;
  convertToPointCloud2(colored_pointcloud, colors, colored_pointcloud_msg);
  colored_pointcloud_msg.header.stamp =
      rclcpp::Time(input->timestamp() * 1e9, rcl_clock_type_t::RCL_ROS_TIME);
  colored_pointcloud_msg.header.frame_id = input->sensorFrameName();

  // 发布点云话题
  colored_point_cloud_pub_->publish(colored_pointcloud_msg);
}

bool PanopticMapper::saveMap(const std::string& file_path) {
  // 保存地图时先 finish，否则保存的地图可能有问题
  map_manager_->finishMapping(submaps_.get());

  bool success = submaps_->saveToFile(file_path);
  LOG_IF(INFO, success) << "Successfully saved " << submaps_->size()
                        << " submaps to '" << file_path << "'.";

  saveIsoSurfacePoints(file_path + "_point_label_cloud.csv");

  if (image_data_manager_) {
    image_data_manager_->getAndRemoveSubmapUnderLock(*submaps_);
    image_data_manager_->saveMappingsToFile(file_path +
                                            "_images_meta_infos.bin");
  }
  return success;
}

bool PanopticMapper::saveIsoSurfacePoints(const std::string& file_path) {
  // 保存物体表面点云
  std::fstream point_label_cloud_file;
  LOG(INFO) << "save to: " << file_path;
  point_label_cloud_file.open(file_path, std::fstream::out);
  if (!point_label_cloud_file.is_open()) {
    LOG(ERROR) << "Could not open file '" << file_path
               << "' to save point label cloud.";
    return false;
  }
  point_label_cloud_file << "x,y,z,id,label,changeStatus,changeStatusId"
                         << std::endl;
  for (const auto& submap : *submaps_) {
    if (submap.getChangeState() == ChangeState::kAbsent ||
        submap.getLabel() == PanopticLabel::kFreeSpace) {
      continue;
    }
    const std::vector<IsoSurfacePoint>& surface_points =
        submap.getIsoSurfacePoints();
    for (const IsoSurfacePoint& point : surface_points) {
      point_label_cloud_file << point.position.x() << "," << point.position.y()
                             << "," << point.position.z() << ","
                             << submap.getID() << "," << submap.getName() << ","
                             << changeStateToString(submap.getChangeState())
                             << "," << static_cast<int>(submap.getChangeState())
                             << std::endl;
    }
  }
  point_label_cloud_file.close();
  return true;
}

bool PanopticMapper::loadMap(const std::string& file_path) {
  auto loaded_map = std::make_shared<SubmapCollection>();

  // Load the map.
  if (!loaded_map->loadFromFile(file_path, true)) {
    return false;
  }

  // Loaded submaps are 'from the past' so set them to inactive.
  for (Submap& submap : *loaded_map) {
    if (submap.getChangeState() == ChangeState::kAbsent) {
      continue;
    }
    submap.finishActivePeriod();
    submap.setMatchRedetection(true);
    if (config_.load_submaps_conservative) {
      submap.setChangeState(ChangeState::kUnobserved);
    } else {
      submap.setChangeState(ChangeState::kPersistent);
    }

    if (!config_.use_saved_embeddings) {
      submap.setEmbeddingVector(std::vector<float>(), 0.f);
    }
  }

  if (config_.loaded_freespace_stays_active) {
    const int freespace_id = loaded_map->getActiveFreeSpaceSubmapID();
    if (loaded_map->submapIdExists(freespace_id)) {
      Submap* freespace =
          loaded_map->getSubmapPtr(loaded_map->getActiveFreeSpaceSubmapID());
      freespace->setIsActive(true);
    } else {
      LOG(WARNING) << "No active freespace submap found at loading.";
    }
  } else {
    loaded_map->setActiveFreeSpaceSubmapID(-1);
  }

  // Set the map.
  submaps_ = loaded_map;

  // Setup the interfaces that use the new collection.
  setupCollectionDependentMembers();

  // Reproduce the mesh and visualization.
  submap_visualizer_->clearMesh();
  submap_visualizer_->reset();
  submap_visualizer_->visualizeAll(submaps_.get());

  LOG_IF(INFO, config_.verbosity >= 1)
      << "Successfully loaded " << submaps_->size() << " submaps.";
  return true;
}

void PanopticMapper::dataLoggingCallback() {
  data_logger_->writeData(node_->get_clock()->now().seconds(), *submaps_);
}

void PanopticMapper::publishVisualizationCallback() { publishVisualization(); }

bool PanopticMapper::setVisualizationModeCallback(
    const panoptic_mapping_msgs::srv::SetVisualizationMode::Request::SharedPtr
        request,
    panoptic_mapping_msgs::srv::SetVisualizationMode::Response::SharedPtr
        response) {
  response->visualization_mode_set = false;
  response->color_mode_set = false;
  bool success = true;

  // Set the visualization mode if requested.
  if (!request->visualization_mode.empty()) {
    SubmapVisualizer::VisualizationMode visualization_mode =
        SubmapVisualizer::visualizationModeFromString(
            request->visualization_mode);
    submap_visualizer_->setVisualizationMode(visualization_mode);
    std::string visualization_mode_is =
        SubmapVisualizer::visualizationModeToString(visualization_mode);
    LOG_IF(INFO, config_.verbosity >= 2)
        << "Set visualization mode to '" << visualization_mode_is << "'.";
    response->visualization_mode_set =
        visualization_mode_is == request->visualization_mode;
    if (!response->visualization_mode_set) {
      success = false;
    }
  }

  // Set the color mode if requested.
  if (!request->color_mode.empty()) {
    SubmapVisualizer::ColorMode color_mode =
        SubmapVisualizer::colorModeFromString(request->color_mode);
    submap_visualizer_->setColorMode(color_mode);
    std::string color_mode_is = SubmapVisualizer::colorModeToString(color_mode);
    LOG_IF(INFO, config_.verbosity >= 2)
        << "Set color mode to '" << color_mode_is << "'.";
    response->color_mode_set = color_mode_is == request->color_mode;
    if (!response->color_mode_set) {
      success = false;
    }
  }

  // Republish the visualization.
  submap_visualizer_->visualizeAll(submaps_.get());
  return success;
}

bool PanopticMapper::saveMapCallback(
    const panoptic_mapping_msgs::srv::SaveLoadMap::Request::SharedPtr request,
    panoptic_mapping_msgs::srv::SaveLoadMap::Response::SharedPtr response) {
  std::lock_guard<std::mutex> lock(node_mutex_);
  response->success = saveMap(request->file_path);
  return response->success;
}

bool PanopticMapper::loadMapCallback(
    const panoptic_mapping_msgs::srv::SaveLoadMap::Request::SharedPtr request,
    panoptic_mapping_msgs::srv::SaveLoadMap::Response::SharedPtr response) {
  response->success = loadMap(request->file_path);
  return response->success;
}

bool PanopticMapper::printTimingsCallback(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response) {
  printTimings();
  return true;
}

void PanopticMapper::printTimingsCallback() { printTimings(); }

void PanopticMapper::printTimings() const { LOG(INFO) << Timing::Print(); }

bool PanopticMapper::finishMappingCallback(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response) {
  std::lock_guard<std::mutex> lock(node_mutex_);
  finishMapping();
  return true;
}

bool PanopticMapper::runningSwitchCallback(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response) {
  std::lock_guard<std::mutex> lock(node_mutex_);
  stop_running_ = !stop_running_;
  return true;
}

std::string PanopticMapper::defaultYamlKeyPath(const std::string& key) {
  const std::pair<std::string, std::string>& ns_and_type =
      default_names_and_types_.at(key);
  std::string yaml_key = "/" + ns_and_type.first;

  bool has_key = false;
  for (const auto& kv : root_yaml_) {
    if (kv.first && kv.first.as<std::string>() == ns_and_type.first) {
      has_key = true;
      break;
    }
  }
  if (!has_key) {
    root_yaml_[ns_and_type.first] = YAML::Node();
  }

  if (!ns_and_type.second.empty() &&
      !config_utilities::hasKeyInYamlPath(root_yaml_, yaml_key, "type")) {
    config_utilities::addKeyValueToYaml(root_yaml_, yaml_key, "type",
                                        ns_and_type.second);
  }
  return yaml_key;
}

YAML::Node PanopticMapper::loadYaml() {
  // Yaml file
  std::string yaml_file_path;
  bool got =
      node_->get_parameter_or("config_path", yaml_file_path, std::string(""));

  CHECK(got) << "No config path provided.";
  CHECK_NE(yaml_file_path, "")
      << "No yaml file path provided. Please provide a path to a yaml file "
         "containing all parameters.";

  YAML::Node result;
  try {
    result = YAML::LoadFile(yaml_file_path);
  } catch (const YAML::Exception& e) {
    CHECK(false) << "Error loading yaml file: " << e.what();
  }

  return result;
}

bool PanopticMapper::removeSubmapCallback(
    const panoptic_mapping_msgs::srv::RemoveSubmap::Request::SharedPtr request,
    panoptic_mapping_msgs::srv::RemoveSubmap::Response::SharedPtr response) {
  std::lock_guard<std::mutex> lock(node_mutex_);

  std::string ids_str = request->submap_ids;
  std::stringstream ss(ids_str);
  std::string str;

  std::set<int> submap_ids_to_remove;
  while (std::getline(ss, str, ',')) {
    if (!str.empty()) {
      try {
        int id = std::stoi(str);
        submap_ids_to_remove.insert(id);
      } catch (const std::invalid_argument& e) {
        // Ignore invalid arguments (non-integer strings)
      } catch (const std::out_of_range& e) {
        // Ignore out of range integers
      }
    }
  }

  std::stringstream response_ss;
  for (auto id : submap_ids_to_remove) {
    if (submaps_->submapIdExists(id)) {
      submaps_->removeSubmap(id);
      response_ss << id << ",";
    }
  }
  LOG(INFO) << "request to remove submap ids: " << request->submap_ids
            << ", actually removed: " << response_ss.str();
  response->removed_submap_ids = response_ss.str();
  return true;
}

bool PanopticMapper::changeSubmapClassNameCallback(
    const panoptic_mapping_msgs::srv::SubmapClassNameChange::Request::SharedPtr
        request,
    panoptic_mapping_msgs::srv::SubmapClassNameChange::Response::SharedPtr
        response) {
  std::lock_guard<std::mutex> lock(node_mutex_);

  int submap_id = request->submap_id;
  std::string new_class_name = request->new_class_name;

  response->success = false;
  if (submaps_->submapIdExists(submap_id)) {
    Submap* submap = submaps_->getSubmapPtr(submap_id);
    submap->setClassName(new_class_name);
    submap->setName(new_class_name);
    submap_visualizer_->changeSubmapVisInfo(*submap);
    response->success = true;
  }
  return true;
}

}  // namespace panoptic_mapping
