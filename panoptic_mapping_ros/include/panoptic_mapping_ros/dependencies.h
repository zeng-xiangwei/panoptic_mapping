#ifndef PANOPTIC_MAPPING_ROS_DEPENDENCIES_H_
#define PANOPTIC_MAPPING_ROS_DEPENDENCIES_H_

#include <panoptic_mapping/integration/class_projective_tsdf_integrator.h>
#include <panoptic_mapping/integration/projection_interpolators.h>
#include <panoptic_mapping/integration/projective_tsdf_integrator.h>
#include <panoptic_mapping/integration/single_tsdf_integrator.h>
#include <panoptic_mapping/labels/csv_label_handler.h>
#include <panoptic_mapping/labels/mini_csv_label_handler.h>
#include <panoptic_mapping/labels/null_label_handler.h>
#include <panoptic_mapping/labels/range_label_handler.h>
#include <panoptic_mapping/map/classification/binary_count.h>
#include <panoptic_mapping/map/classification/fixed_count.h>
#include <panoptic_mapping/map/classification/moving_binary_count.h>
#include <panoptic_mapping/map/classification/uncertainty.h>
#include <panoptic_mapping/map/classification/variable_count.h>
#include <panoptic_mapping/map/scores/average.h>
#include <panoptic_mapping/map/scores/latest.h>
#include <panoptic_mapping/map_management/map_manager.h>
#include <panoptic_mapping/map_management/null_map_manager.h>
#include <panoptic_mapping/submap_allocation/monolithic_freespace_allocator.h>
#include <panoptic_mapping/submap_allocation/null_submap_allocator.h>
#include <panoptic_mapping/submap_allocation/semantic_submap_allocator.h>
#include <panoptic_mapping/tools/evaluation_data_writer.h>
#include <panoptic_mapping/tools/log_data_writer.h>
#include <panoptic_mapping/tools/null_data_writer.h>
#include <panoptic_mapping/tracking/detectron_id_tracker.h>
#include <panoptic_mapping/tracking/ground_truth_id_tracker.h>
#include <panoptic_mapping/tracking/projective_id_tracker.h>
#include <panoptic_mapping/tracking/single_tsdf_tracker.h>

// 考虑到panoptic_mapping是静态库时，可能不会把所有符号都链接过来，
// 仅会链接已使用的符号，因此无法保证工厂方法的使用，
// 需要在这里把可能用到的符号写在这里
namespace {
using namespace panoptic_mapping;
// Integration
volatile void* dummy_class_projective_tsdf_integrator_registration =
    &ClassProjectiveIntegrator::registration_;
volatile void* dummy_interpolator_nearest_registration =
    &InterpolatorNearest::registration_;
volatile void* dummy_interpolator_bilinear_registration =
    &InterpolatorBilinear::registration_;
volatile void* dummy_interpolator_adaptive_registration =
    &InterpolatorAdaptive::registration_;
volatile void* dummy_projective_integrator =
    &ProjectiveIntegrator::registration_;
volatile void* dummy_single_tsdf_integrator =
    &SingleTsdfIntegrator::registration_;

// Labels
volatile void* dummy_csv_label_handler_registration =
    &CsvLabelHandler::registration_;
volatile void* dummy_mini_csv_label_handler_registration =
    &MiniCsvLabelHandler::registration_;
volatile void* dummy_null_label_handler_registration =
    &NullLabelHandler::registration_;
volatile void* dummy_range_label_handler_registration =
    &RangeLabelHandler::registration_;

// Classification
volatile void* dummy_binary_count_registration =
    &BinaryCountLayer::registration_;
volatile void* dummy_fixed_count_registration = &FixedCountLayer::registration_;
volatile void* dummy_moving_binary_count_registration =
    &MovingBinaryCountLayer::registration_;
volatile void* dummy_uncertainty_registration =
    &UncertaintyLayer::registration_;
volatile void* dummy_variable_count_registration =
    &VariableCountLayer::registration_;

// Scores
volatile void* dummy_average_score_registration =
    &AverageScoreLayer::registration_;
volatile void* dummy_latest_score_registration =
    &LatestScoreLayer::registration_;

// Map Management
volatile void* dummy_map_manager_registration = &MapManager::registration_;
volatile void* dummy_null_map_manager_registration =
    &NullMapManager::registration_;

// Submap Allocation
volatile void* dummy_monolithic_freespace_allocator_registration =
    &MonolithicFreespaceAllocator::registration_;
volatile void* dummy_null_submap_allocator_registration =
    &NullSubmapAllocator::registration_;
volatile void* dummy_null_freespace_allocator =
    &NullFreespaceAllocator::registration_;
volatile void* dummy_semantic_submap_allocator_registration =
    &SemanticSubmapAllocator::registration_;

// Data Writers
volatile void* dummy_evaluation_data_writer_registration =
    &EvaluationDataWriter::registration_;
volatile void* dummy_log_data_writer_registration =
    &LogDataWriter::registration_;
volatile void* dummy_null_data_writer_registration =
    &NullDataWriter::registration_;

// Tracking
volatile void* dummy_detectron_id_tracker_registration =
    &DetectronIDTracker::registration_;
volatile void* dummy_ground_truth_id_tracker_registration =
    &GroundTruthIDTracker::registration_;
volatile void* dummy_projective_id_tracker_registration =
    &ProjectiveIDTracker::registration_;
volatile void* dummy_single_tsdf_tracker_registration =
    &SingleTSDFTracker::registration_;
}  // namespace

#endif