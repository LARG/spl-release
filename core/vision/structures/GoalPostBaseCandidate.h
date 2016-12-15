#pragma once

#include <iostream>

/// @ingroup vision
class GoalPostBaseCandidate {

public:
  GoalPostBaseCandidate() = default;
  ~GoalPostBaseCandidate() = default;

  // Vision/Localization properties 
  uint16_t xi, xf, yi, yf;

  bool invalid = false;

  static bool is_contained(const GoalPostBaseCandidate &to_check, const GoalPostBaseCandidate &parent);
  static bool almost_identical(const GoalPostBaseCandidate &to_check, const GoalPostBaseCandidate &parent);
  static bool is_overlapping(const GoalPostBaseCandidate &to_check, const GoalPostBaseCandidate &parent);
  static bool is_close(const GoalPostBaseCandidate &to_check, const GoalPostBaseCandidate &parent);

};
