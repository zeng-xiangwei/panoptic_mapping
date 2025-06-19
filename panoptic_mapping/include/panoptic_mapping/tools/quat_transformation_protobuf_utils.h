#ifndef PANOPTIC_MAPPING_TOOLS_QUAT_TRANSFORMATION_PROTOBUF_UTILS_H_
#define PANOPTIC_MAPPING_TOOLS_QUAT_TRANSFORMATION_PROTOBUF_UTILS_H_

#include "panoptic_mapping/QuatTransformation.pb.h"
#include "panoptic_mapping/common/common.h"

namespace panoptic_mapping {
namespace conversions {

void transformKindrToProto(const Transformation& transformation,
                           QuatTransformationProto* quat_transformation_proto);

void transformProtoToKindr(
    const QuatTransformationProto& quat_transformation_proto,
    Transformation* transformation);

}  // namespace conversions
}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_QUAT_TRANSFORMATION_PROTOBUF_UTILS_H_
