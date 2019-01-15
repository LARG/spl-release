#include <vision/structures/ObjectCandidate.h>


bool ObjectCandidate::sortByXi(const ObjectCandidate &left, const ObjectCandidate &right){
  return left.xi <= right.xi;
}
