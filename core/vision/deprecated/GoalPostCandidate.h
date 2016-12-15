#pragma once

#include <iostream>
#include <vector>

/// @ingroup vision
struct GoalPostCandidate {

  // Vision/Localization properties 

  // bottom corners: (x0, y0), (x1, y1) left to right
  // top corners: (x2, y2), (x3, y3) left to right
  uint16_t x0, x1, x2, x3, y0, y1, y2, y3;
  uint16_t avgX, avgY;

  // Features
  float whitePct;
  float diff = 0.0;
  float difflocX;
  float difflocY;
  float width;              // Estimated world width of the post

  std::vector<std::pair<std::pair<int, int>, int>> diffs;

  bool invalid = false;

};
