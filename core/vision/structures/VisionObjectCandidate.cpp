#include <vision/structures/VisionObjectCandidate.h>


bool VisionObjectCandidate::sortByXi(const VisionObjectCandidate &left, const VisionObjectCandidate &right){
  return left.xi <= right.xi;
}
