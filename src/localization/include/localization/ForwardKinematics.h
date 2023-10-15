#ifndef FORWARDKINEMATICS_GJA13VPQ
#define FORWARDKINEMATICS_GJA13VPQ

#include <vector>
#include "localization/Pose3D.h"
#include "localization/RobotDimensions.h"

namespace ForwardKinematics {
  using std::vector;
  std::vector<Pose3D> calculateRelativePose(
      const std::vector<float> &joint_angles,
      const std::array<float, RobotDimensions::NUM_DIMENSIONS> &dimensions);
  std::vector<Pose3D> calculateAbsolutePose(
      const Pose3D torso, const std::vector<Pose3D> &rel_parts);
  Pose3D calculateVirtualBase(bool useLeft,
      const std::vector<Pose3D> &rel_parts);
}  // namespace ForwardKinematics

#endif /* end of include guard: FORWARDKINEMATICS_GJA13VPQ */
