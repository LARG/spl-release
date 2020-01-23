#pragma once

#include "Position.h"
struct VisionObjectCandidate {

  uint16_t xi, xf, dx, yi, yf, dy;
  uint16_t avgX, avgY;
  bool valid;
  uint16_t hullY;
  float distToEdge;
  Position pos;

  bool robotCandidate;
  bool postCandidate;


  static bool sortByXi(const VisionObjectCandidate &left, const VisionObjectCandidate &right);

};
