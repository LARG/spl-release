#include <vision/structures/GoalPostBaseCandidate.h>

bool GoalPostBaseCandidate::is_contained(const GoalPostBaseCandidate &c1, const GoalPostBaseCandidate &c2) {
  if (c1.xi >= c2.xi && c1.xf <= c2.xf) {
    return true;
  }

  return false;
}

bool GoalPostBaseCandidate::almost_identical(const GoalPostBaseCandidate &c1, const GoalPostBaseCandidate &c2) {
  int width1 = c1.xf - c1.xi;
  int width2 = c2.xf - c2.xi;
  if (abs(c1.xi - c2.xi) <= 30 && abs(width1 - width2) <= 60) {
    return true;
  }

  return false;
}

bool GoalPostBaseCandidate::is_overlapping(const GoalPostBaseCandidate &c1, const GoalPostBaseCandidate &c2) {
  if (c1.xf >= c2.xi && c1.xf <= c2.xf) {
    return true;
  }

  return false;
}

bool GoalPostBaseCandidate::is_close(const GoalPostBaseCandidate &c1, const GoalPostBaseCandidate &c2) {
  if (abs(c1.xf - c2.xi) < 35) {
    return true;
  }

  return false;
}
