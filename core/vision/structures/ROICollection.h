#include <vision/structures/ROI.h>
#include <vector>

struct ROICollection {
  std::vector<ROI> top;
  std::vector<ROI> bottom;
};
