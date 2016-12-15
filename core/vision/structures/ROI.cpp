#include <vision/structures/ROI.h>
#include <vision/ColorSegmenter.h>

#ifdef TOOL
void ROI::extract() const {
  segmenter->extractMat(*this);
  this->extracted = true;
}
#endif

std::ostream& operator<<(std::ostream& os, const ROI& roi) {
  os << roi.str();
  return os;
}
