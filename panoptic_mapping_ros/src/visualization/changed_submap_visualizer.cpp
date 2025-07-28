#include "panoptic_mapping_ros/visualization/changed_submap_visualizer.h"

namespace panoptic_mapping {

namespace {
const Color kAddColor = Color::Green();
const Color kDeletedColor = Color::Red();
const Color kChangedColor = Color::Yellow();
const Color kUnchangedColor = Color::Blue();
}  // namespace
void ChangedSubmapVisualizer::Config::checkParams() const {}

void ChangedSubmapVisualizer::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("change_point_threshold", &change_point_threshold);
  setupParam("only_use_aabb", &only_use_aabb);
  setupParam("min_obb_length", &min_obb_length);
  setupParam("obb_frame_id", &obb_frame_id);
  setupParam("use_redetection_for_add", &use_redetection_for_add);
  setupParam("box_only_align_z", &box_only_align_z);
}

ChangedSubmapVisualizer::ChangedSubmapVisualizer(const Config& config,
                                                 rclcpp::Node::SharedPtr node)
    : config_(config.checkValid()), node_(node) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  obb_publisher_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "visualization/submaps/changed_submaps", rclcpp::QoS(10));

  #ifdef VLN_MSGS_FOUND
  vln_map_update_pub_ = node_->create_publisher<vln_msgs::msg::MapUpdate>(
          "vln_map_update", rclcpp::QoS(10));
  #endif
}

void ChangedSubmapVisualizer::visualizeChangedSubmaps(
    SubmapCollection* submaps) {
  // 检测变化的物体
  findChangedSubmaps(*submaps);

  // 发布变化
  publishChanges(*submaps);
  publishChangesForVln(*submaps);

  // 删除 kDeleted 的数据
  update();
}

void ChangedSubmapVisualizer::reset() {
  submap_infos_.clear();
  previous_submaps_ = nullptr;
}

void ChangedSubmapVisualizer::update() {
  for (auto it = submap_infos_.begin(); it != submap_infos_.end();) {
    if (it->second.change_type == ChangeType::kDeleted) {
      it = submap_infos_.erase(it);
    } else {
      ++it;
    }
  }
}

void ChangedSubmapVisualizer::findChangedSubmaps(SubmapCollection& submaps) {
  if (previous_submaps_ != &submaps) {
    reset();
    previous_submaps_ = &submaps;
    LOG(INFO) << "submaps address changed";
  }

  // Update submap ids.
  std::vector<int> ids;
  std::vector<int> new_ids;
  std::vector<int> deleted_ids;
  ids.reserve(submap_infos_.size());
  for (const auto& id_info_pair : submap_infos_) {
    ids.emplace_back(id_info_pair.first);
  }
  submaps.updateIDList(ids, &new_ids, &deleted_ids);

  // New submaps.
  for (int id : new_ids) {
    Submap& submap = *(submaps.getSubmapPtr(id));
    if (submap.getChangeState() == ChangeState::kAbsent ||
        submap.getLabel() == PanopticLabel::kFreeSpace) {
      continue;
    }
    if (config_.use_redetection_for_add && !submap.matchRedetection()) {
      continue;
    }

    if (submap.getChangeState() == ChangeState::kNew) {
      submap.computeIsoSurfacePoints();
    }
    auto it = submap_infos_.emplace(std::make_pair(id, SubmapInfo())).first;
    SubmapInfo& info = it->second;
    info.id = id;
    info.name = submap.getClassName();
    info.change_type = ChangeType::kAdded;
    info.surface_points_size = submap.getIsoSurfacePoints().size();
    info.obb = computeOBB(submap.getIsoSurfacePoints());
    info.color = kAddColor;
    info.embedding_vector = submap.getEmbeddingVector();
  }

  // Deleted Submaps.
  for (int id : deleted_ids) {
    submap_infos_[id].change_type = ChangeType::kDeleted;
    submap_infos_[id].color = kDeletedColor;
  }

  // Check updated Submaps in old.
  for (int id : ids) {
    if (submap_infos_[id].change_type == ChangeType::kDeleted) {
      continue;
    }

    SubmapInfo& info = submap_infos_[id];
    Submap& submap = *(submaps.getSubmapPtr(id));
    if (submap.getChangeState() == ChangeState::kAbsent) {
      info.change_type = ChangeType::kDeleted;
      info.color = kDeletedColor;
      continue;
    }

    if (submap.getChangeState() == ChangeState::kNew) {
      submap.computeIsoSurfacePoints();
    }

    int surface_points_size = submap.getIsoSurfacePoints().size();
    if (std::abs(surface_points_size - info.surface_points_size) >
        config_.change_point_threshold) {
      if (config_.verbosity >= 4) {
        LOG(INFO) << "Submap " << id << " surface points changes "
                  << info.surface_points_size << " -> " << surface_points_size;
      }
      info.change_type = ChangeType::kChanged;
      info.obb = computeOBB(submap.getIsoSurfacePoints());
      info.color = kChangedColor;
      info.surface_points_size = surface_points_size;
      info.embedding_vector = submap.getEmbeddingVector();
    } else {
      info.change_type = ChangeType::kUnChanged;
      info.color = kUnchangedColor;
    }
  }

  if (config_.verbosity >= 4) {
    std::stringstream ss_add, ss_del, ss_change, ss_unchange;
    int add_count = 0, del_count = 0, change_count = 0, unchange_count = 0;
    for (auto& kv : submap_infos_) {
      switch (kv.second.change_type) {
        case ChangeType::kAdded:
          add_count++;
          ss_add << kv.first << " ";
          break;
        case ChangeType::kDeleted:
          del_count++;
          ss_del << kv.first << " ";
          break;
        case ChangeType::kChanged:
          change_count++;
          ss_change << kv.first << " ";
          break;
        case ChangeType::kUnChanged:
          unchange_count++;
          ss_unchange << kv.first << " ";
          break;
        default:
          break;
      }
    }
    LOG(INFO) << "Added: " << add_count << " submaps, ids: " << ss_add.str();
    LOG(INFO) << " Deleted: " << del_count << " submaps, ids: " << ss_del.str();
    LOG(INFO) << " Changed: " << change_count
              << " submaps, ids: " << ss_change.str();
    LOG(INFO) << " Unchanged: " << unchange_count
              << " submaps, ids: " << ss_unchange.str();
  }
}

void ChangedSubmapVisualizer::publishChanges(const SubmapCollection& submaps) {
  visualization_msgs::msg::MarkerArray result;
  for (auto& kv : submap_infos_) {
    const SubmapInfo& info = kv.second;
    if (config_.verbosity >= 4 && !info.obb.valid) {
      LOG(WARNING) << " Submap " << kv.first << " has an invalid bounding box.";
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = config_.obb_frame_id;
    marker.header.stamp = node_->get_clock()->now();
    marker.color.r = info.color.r;
    marker.color.g = info.color.g;
    marker.color.b = info.color.b;
    marker.color.a = 255;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.id = info.id;
    marker.ns =
        info.name + "_" + std::to_string(info.id) + "_" + info.obb.box_type;
    marker.scale.x = info.obb.extents(0);
    marker.scale.y = info.obb.extents(1);
    marker.scale.z = info.obb.extents(2);
    Eigen::Quaternionf q(info.obb.rotation);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.pose.position.x = info.obb.center.x();
    marker.pose.position.y = info.obb.center.y();
    marker.pose.position.z = info.obb.center.z();

    if (info.change_type == ChangeType::kUnChanged) {
      marker.action = visualization_msgs::msg::Marker::MODIFY;
      if (info.obb.valid) {
        result.markers.push_back(marker);
      }
      continue;
    }

    int submap_id = info.id;
    if (info.change_type == ChangeType::kAdded) {
      marker.action = visualization_msgs::msg::Marker::ADD;
    } else if (info.change_type == ChangeType::kDeleted) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    } else if (info.change_type == ChangeType::kChanged) {
      marker.action = visualization_msgs::msg::Marker::MODIFY;
    }

    if (info.obb.valid) {
      result.markers.push_back(marker);
    }
  }

  obb_publisher_->publish(result);
}

void ChangedSubmapVisualizer::publishChangesForVln(const SubmapCollection& submaps) {
  #ifdef VLN_MSGS_FOUND
  vln_msgs::msg::MapUpdate result;
  for (auto& kv : submap_infos_) {
    const SubmapInfo& info = kv.second;
    if (config_.verbosity >= 4 && !info.obb.valid) {
      LOG(WARNING) << " Submap " << kv.first << " has an invalid bounding box.";
    }

    if (info.change_type == ChangeType::kUnChanged || !info.obb.valid) {
      continue;
    }

    int submap_id = info.id;
    vln_msgs::msg::SemanticObject obj;
    obj.id = submap_id;
    obj.name = info.name;
    obj.center.x = info.obb.center.x();
    obj.center.y = info.obb.center.y();
    obj.center.z = info.obb.center.z();
    obj.length = info.obb.extents(0);
    obj.width = info.obb.extents(1);
    obj.height = info.obb.extents(2);
    Eigen::Quaternionf q(info.obb.rotation);
    obj.quat.x = q.x();
    obj.quat.y = q.y();
    obj.quat.z = q.z();
    obj.quat.w = q.w();
    obj.embedding_vector = info.embedding_vector;
    if (info.change_type == ChangeType::kAdded) {
      result.add_objects.push_back(obj);
    } else if (info.change_type == ChangeType::kDeleted) {
      result.del_objects.push_back(obj);
    } else if (info.change_type == ChangeType::kChanged) {
      result.update_objects.push_back(obj);
    }
  }

  vln_map_update_pub_->publish(result);
  #endif
}

ChangedSubmapVisualizer::OrientedBoundingBox
ChangedSubmapVisualizer::computeOBB(
    const std::vector<IsoSurfacePoint>& points) {
  if (config_.box_only_align_z) {
    return computeZAlignedOBB(points);
  }
  return computeStandardOBB(points);
}

ChangedSubmapVisualizer::OrientedBoundingBox
ChangedSubmapVisualizer::computeStandardOBB(
    const std::vector<IsoSurfacePoint>& points) {
  const float kEpsilon = 1e-6f;

  if (points.size() < 3) {
    OrientedBoundingBox result = OrientedBoundingBox();
    return result;
  }

  // Step 1: Compute the centroid of the points
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (const auto& pt : points) {
    centroid += pt.position;
  }
  centroid /= static_cast<float>(points.size());

  // Step 2: Compute the covariance matrix
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (const auto& pt : points) {
    Eigen::Vector3f diff = pt.position - centroid;
    cov += diff * diff.transpose();
  }

  // Step 3: Perform eigen decomposition to get principal axes
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
  Eigen::Vector3f eigenvalues = solver.eigenvalues();
  Eigen::Matrix3f eigenvectors = solver.eigenvectors();

  OrientedBoundingBox obb;

  // 特征值较小时，使用AABB包围盒
  if (config_.only_use_aabb || eigenvalues(0) < kEpsilon ||
      eigenvalues(1) < kEpsilon || eigenvalues(2) < kEpsilon) {
    Eigen::Vector3f min_pt, max_pt;
    min_pt = max_pt = points[0].position;

    for (const auto& pt : points) {
      const Eigen::Vector3f& pt_pos = pt.position;
      for (int i = 0; i < 3; ++i) {
        if (pt_pos(i) < min_pt(i)) min_pt(i) = pt_pos(i);
        if (pt_pos(i) > max_pt(i)) max_pt(i) = pt_pos(i);
      }
    }

    obb.center = centroid;
    obb.extents = (max_pt - min_pt);
    obb.rotation = Eigen::Matrix3f::Identity();
    obb.box_type = "AABB";
  } else {
    // 正常 OBB 计算
    Eigen::Vector3f min_proj =
        Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f max_proj =
        Eigen::Vector3f::Constant(-std::numeric_limits<float>::max());

    for (const auto& pt : points) {
      Eigen::Vector3f proj =
          eigenvectors.transpose() * (pt.position - centroid);
      for (int i = 0; i < 3; ++i) {
        if (proj(i) < min_proj(i)) min_proj(i) = proj(i);
        if (proj(i) > max_proj(i)) max_proj(i) = proj(i);
      }
    }

    obb.center = centroid;
    obb.extents = (max_proj - min_proj);
    obb.rotation = eigenvectors;
    obb.box_type = "OBB";
  }

  for (int i = 0; i < 3; ++i) {
    if (obb.extents(i) < config_.min_obb_length) {
      obb.extents(i) = config_.min_obb_length;
    }
  }
  obb.valid = true;
  return obb;
}

ChangedSubmapVisualizer::OrientedBoundingBox
ChangedSubmapVisualizer::computeZAlignedOBB(
    const std::vector<IsoSurfacePoint>& points) {
  const float kEpsilon = 1e-6f;

  if (points.size() < 3) {
    OrientedBoundingBox result = OrientedBoundingBox();
    return result;
  }

  // Step 1: Compute the centroid of the points in XY plane
  Eigen::Vector2f centroid_xy = Eigen::Vector2f::Zero();
  for (const auto& pt : points) {
    centroid_xy += pt.position.head<2>();
  }
  centroid_xy /= static_cast<float>(points.size());

  // Step 2: Compute the covariance matrix in XY plane
  Eigen::Matrix2f cov_xy = Eigen::Matrix2f::Zero();
  for (const auto& pt : points) {
    Eigen::Vector2f diff = pt.position.head<2>() - centroid_xy;
    cov_xy += diff * diff.transpose();
  }

  // Step 3: Perform eigen decomposition to get principal axes in XY plane
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov_xy);
  Eigen::Vector2f eigenvalues = solver.eigenvalues();
  Eigen::Matrix2f eigenvectors_xy = solver.eigenvectors();

  // Check if eigenvalues are too small, use AABB instead
  if (config_.only_use_aabb || eigenvalues(0) < kEpsilon ||
      eigenvalues(1) < kEpsilon) {
    Eigen::Vector2f min_pt, max_pt;
    min_pt = max_pt = points[0].position.head<2>();

    for (const auto& pt : points) {
      const Eigen::Vector2f& pt_pos = pt.position.head<2>();
      for (int i = 0; i < 2; ++i) {
        if (pt_pos(i) < min_pt(i)) min_pt(i) = pt_pos(i);
        if (pt_pos(i) > max_pt(i)) max_pt(i) = pt_pos(i);
      }
    }

    OrientedBoundingBox obb;
    obb.center.head<2>() = centroid_xy;
    obb.extents.head<2>() = (max_pt - min_pt);
    obb.rotation.block<2, 2>(0, 0) = Eigen::Matrix2f::Identity();
    obb.box_type = "AABB";
    obb.valid = true;
    return obb;
  }

  // Project points onto the XY plane and compute OBB
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();

  // Compute the angle from the principal axis
  float angle = std::atan2(eigenvectors_xy(1, 0), eigenvectors_xy(0, 0));
  rotation.block<2, 2>(0, 0) << std::cos(angle), -std::sin(angle),
      std::sin(angle), std::cos(angle);

  // Calculate extents in XY plane
  Eigen::Vector2f min_proj =
      Eigen::Vector2f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector2f max_proj =
      Eigen::Vector2f::Constant(-std::numeric_limits<float>::max());

  for (const auto& pt : points) {
    Eigen::Vector2f pt_xy = pt.position.head<2>() - centroid_xy;
    Eigen::Vector2f rotated_pt = rotation.block<2, 2>(0, 0).transpose() * pt_xy;
    for (int i = 0; i < 2; ++i) {
      if (rotated_pt(i) < min_proj(i)) min_proj(i) = rotated_pt(i);
      if (rotated_pt(i) > max_proj(i)) max_proj(i) = rotated_pt(i);
    }
  }

  // Extend Z-axis dimensions
  float min_z = std::numeric_limits<float>::max();
  float max_z = -std::numeric_limits<float>::max();
  for (const auto& pt : points) {
    if (pt.position.z() < min_z) min_z = pt.position.z();
    if (pt.position.z() > max_z) max_z = pt.position.z();
  }

  // Build final OBB
  OrientedBoundingBox obb;
  obb.center.head<2>() = centroid_xy;
  obb.center.z() = (min_z + max_z) / 2.0f;
  obb.extents.head<2>() = (max_proj - min_proj);
  obb.extents.z() = max_z - min_z;
  obb.rotation = rotation;
  obb.box_type = "OBB";

  // Apply minimum size constraints
  for (int i = 0; i < 3; ++i) {
    if (obb.extents(i) < config_.min_obb_length) {
      obb.extents(i) = config_.min_obb_length;
    }
  }

  obb.valid = true;
  return obb;
}

}  // namespace panoptic_mapping