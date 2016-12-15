#include <memory/ROIBlock.h>

ROIBlock::ROIBlock() {
  header.version = 0;
  header.size = sizeof(ROIBlock);
}
