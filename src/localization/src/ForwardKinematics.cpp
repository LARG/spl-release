#include <stdio.h>
#include "nao_sensor_msgs/msg/joint_indexes.hpp"
#include "localization/ForwardKinematics.h"
#include "localization/RobotDimensions.h"

using nao_sensor_msgs::msg::JointIndexes;

std::vector<Pose3D> ForwardKinematics::calculateRelativePose(
    const std::vector<float> &joint_angles,
    const std::array<float, RobotDimensions::NUM_DIMENSIONS> &dimensions) {
  std::vector<Pose3D> rel_parts(BodyPart::NUM_PARTS);

  rel_parts[BodyPart::torso] = Pose3D(0, 0, 0);

  // HEAD
  rel_parts[BodyPart::neck] = Pose3D(rel_parts[BodyPart::torso])
    .translate(0, 0, dimensions[RobotDimensions::torsoToHeadPan])
    .rotateZ(joint_angles[JointIndexes::HEADYAW]);

  rel_parts[BodyPart::head] = Pose3D(rel_parts[BodyPart::neck])
    .rotateY(joint_angles[JointIndexes::HEADPITCH]);

  rel_parts[BodyPart::top_camera] = Pose3D(rel_parts[BodyPart::head])
    .translate(dimensions[RobotDimensions::xHeadTiltToTopCamera], 0,
        dimensions[RobotDimensions::zHeadTiltToTopCamera])
    .rotateY(dimensions[-RobotDimensions::tiltOffsetToTopCamera])
    .rotateX(dimensions[RobotDimensions::rollOffsetToTopCamera])
    .rotateZ(dimensions[RobotDimensions::yawOffsetToTopCamera]);

  rel_parts[BodyPart::bottom_camera] = Pose3D(rel_parts[BodyPart::head])
    .translate(dimensions[RobotDimensions::xHeadTiltToBottomCamera], 0,
        dimensions[RobotDimensions::zHeadTiltToBottomCamera])
    .rotateY(dimensions[RobotDimensions::tiltOffsetToBottomCamera])
    .rotateX(dimensions[RobotDimensions::rollOffsetToBottomCamera])
    .rotateZ(dimensions[RobotDimensions::yawOffsetToBottomCamera]);

  // ARMS
  rel_parts[BodyPart::left_shoulder] = Pose3D(rel_parts[BodyPart::torso])
    .translate(dimensions[RobotDimensions::armOffset1],
        dimensions[RobotDimensions::armOffset2],
        dimensions[RobotDimensions::armOffset3])
    .rotateY(-joint_angles[JointIndexes::LSHOULDERPITCH]);
  rel_parts[BodyPart::right_shoulder] = Pose3D(rel_parts[BodyPart::torso])
    .translate(dimensions[RobotDimensions::armOffset1],
        -dimensions[RobotDimensions::armOffset2],
        dimensions[RobotDimensions::armOffset3])
    .rotateY(-joint_angles[JointIndexes::RSHOULDERPITCH]);

  rel_parts[BodyPart::left_bicep] = Pose3D(rel_parts[BodyPart::left_shoulder])
    .rotateZ(joint_angles[JointIndexes::LSHOULDERROLL]);
  rel_parts[BodyPart::right_bicep] = Pose3D(rel_parts[BodyPart::right_shoulder])
    .rotateZ(-joint_angles[JointIndexes::RSHOULDERROLL]);

  rel_parts[BodyPart::left_elbow] = Pose3D(rel_parts[BodyPart::left_bicep])
    .translate(dimensions[RobotDimensions::upperArmLength],
        dimensions[RobotDimensions::elbowOffsetY], 0)
    .rotateX(joint_angles[JointIndexes::LELBOWYAW]);
  rel_parts[BodyPart::right_elbow] = Pose3D(rel_parts[BodyPart::right_bicep])
    .translate(dimensions[RobotDimensions::upperArmLength],
        dimensions[RobotDimensions::elbowOffsetY], 0)
    .rotateX(-joint_angles[JointIndexes::RELBOWYAW]);

  rel_parts[BodyPart::left_forearm] = Pose3D(rel_parts[BodyPart::left_elbow])
    .rotateZ(joint_angles[JointIndexes::LELBOWROLL]);
  rel_parts[BodyPart::right_forearm] = Pose3D(rel_parts[BodyPart::right_elbow])
    .rotateZ(-joint_angles[JointIndexes::RELBOWROLL]);

  rel_parts[BodyPart::left_hand] = Pose3D(rel_parts[BodyPart::left_forearm])
    .translate(dimensions[RobotDimensions::lowerArmLength], 0, 0);
  rel_parts[BodyPart::right_hand] = Pose3D(rel_parts[BodyPart::right_forearm])
    .translate(dimensions[RobotDimensions::lowerArmLength], 0, 0);

  // LEGS
  rel_parts[BodyPart::left_pelvis] = Pose3D(rel_parts[BodyPart::torso])
    .rotateX(dimensions[RobotDimensions::torsoHipRoll])
    .translate(0, 0, -dimensions[RobotDimensions::torsoToHip])
    .rotateZ(-joint_angles[JointIndexes::LHIPYAWPITCH]);
  rel_parts[BodyPart::right_pelvis] = Pose3D(rel_parts[BodyPart::torso])
    .rotateX(-dimensions[RobotDimensions::torsoHipRoll])
    .translate(0, 0, -dimensions[RobotDimensions::torsoToHip])
    .rotateZ(joint_angles[JointIndexes::LHIPYAWPITCH]);

  rel_parts[BodyPart::left_hip] = Pose3D(rel_parts[BodyPart::left_pelvis])
    .rotateX(joint_angles[JointIndexes::LHIPROLL] -
             dimensions[RobotDimensions::torsoHipRoll]);
  rel_parts[BodyPart::right_hip] = Pose3D(rel_parts[BodyPart::right_pelvis])
    .rotateX(joint_angles[JointIndexes::RHIPROLL] +
             dimensions[RobotDimensions::torsoHipRoll]);

  rel_parts[BodyPart::left_thigh] = Pose3D(rel_parts[BodyPart::left_hip])
    .rotateY(joint_angles[JointIndexes::LHIPPITCH]);
  rel_parts[BodyPart::right_thigh] = Pose3D(rel_parts[BodyPart::right_hip])
    .rotateY(joint_angles[JointIndexes::RHIPPITCH]);

  rel_parts[BodyPart::left_tibia] = Pose3D(rel_parts[BodyPart::left_thigh])
    .translate(0, 0, -dimensions[RobotDimensions::upperLegLength])
    .rotateY(joint_angles[JointIndexes::LKNEEPITCH]);
  rel_parts[BodyPart::right_tibia] = Pose3D(rel_parts[BodyPart::right_thigh])
    .translate(0, 0, -dimensions[RobotDimensions::upperLegLength])
    .rotateY(joint_angles[JointIndexes::RKNEEPITCH]);

  rel_parts[BodyPart::left_ankle] = Pose3D(rel_parts[BodyPart::left_tibia])
    .translate(0, 0, -dimensions[RobotDimensions::lowerLegLength])
    .rotateY(joint_angles[JointIndexes::LANKLEPITCH]);
  rel_parts[BodyPart::right_ankle] = Pose3D(rel_parts[BodyPart::right_tibia])
    .translate(0, 0, -dimensions[RobotDimensions::lowerLegLength])
    .rotateY(joint_angles[JointIndexes::RANKLEPITCH]);

  rel_parts[BodyPart::left_foot] = Pose3D(rel_parts[BodyPart::left_ankle])
    .rotateX(-joint_angles[JointIndexes::LANKLEROLL]);
  rel_parts[BodyPart::right_foot] = Pose3D(rel_parts[BodyPart::right_ankle])
    .rotateX(joint_angles[JointIndexes::RANKLEROLL]);

  rel_parts[BodyPart::left_sole] = Pose3D(rel_parts[BodyPart::left_foot])
    .translate(0, 0, -dimensions[RobotDimensions::footHeight]);
  rel_parts[BodyPart::right_sole] = Pose3D(rel_parts[BodyPart::right_foot])
    .translate(0, 0, -dimensions[RobotDimensions::footHeight]);

  return rel_parts;
}

Pose3D ForwardKinematics::calculateVirtualBase(
    bool useLeft, const std::vector<Pose3D> &rel_parts) {
  Pose3D baseFoot;
  if (useLeft)
    baseFoot = rel_parts[BodyPart::left_sole];
  else
    baseFoot = rel_parts[BodyPart::right_sole];

  // Get the torso in the foot's coordinate frame
  Pose3D torsoInFootFrame = rel_parts[BodyPart::torso].relativeTo(baseFoot);
  // In the foot frame, the base is offset by the torso's XY translation
  Pose3D baseInFootFrame(torsoInFootFrame.translation(0),
      torsoInFootFrame.translation(1), 0);
  // In the foot frame, the base is rotated by the torso's Z rotation
  baseInFootFrame.rotateZ(torsoInFootFrame.rotation
      .toRotationMatrix().eulerAngles(2, 1, 0)(0));
  // Make the base relative to the torso
  Pose3D base = baseInFootFrame.relativeTo(torsoInFootFrame);

  return base;
}

std::vector<Pose3D> ForwardKinematics::calculateAbsolutePose(
    const Pose3D torso, const std::vector<Pose3D> &rel_parts) {
  std::vector<Pose3D> abs_parts(BodyPart::NUM_PARTS);
  for (int i = 0; i < BodyPart::NUM_PARTS; i++)
    abs_parts[i] = rel_parts[i].relativeTo(torso);
  return abs_parts;
}
