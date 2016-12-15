#pragma once

#include <memory/MemoryBlock.h>
#include <vision/structures/ROI.h>
#include <schema/gen/ROIBlock_generated.h>

DECLARE_INTERNAL_SCHEMA(class ROIBlock : public MemoryBlock {
public:
  SCHEMA_METHODS(ROIBlock);
  ROIBlock();
  SCHEMA_FIELD(std::vector<ROI> ball_rois_);
  SCHEMA_FIELD(std::vector<ROI> goal_rois_);
  SCHEMA_POST_DESERIALIZATION({
    for(auto& roi : ball_rois_)
      roi.segmenter = nullptr;
    for(auto& roi : goal_rois_)
      roi.segmenter = nullptr;
  });
});
