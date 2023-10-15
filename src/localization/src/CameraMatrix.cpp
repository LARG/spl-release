#include <iostream>
#include <stdexcept>
#include "localization/CameraMatrix.h"

using Eigen::Vector3d;
using Eigen::Vector4d;

CameraMatrix::CameraMatrix(const bool isTopCamera,
    const int imageWidth, const int imageHeight):
  imageWidth_(imageWidth),
  imageHeight_(imageHeight),
  isTopCamera_(isTopCamera) {
  setCalibration();
}

WorldPosition CameraMatrix::getWorldPosition(
    float imageX, float imageY, float height) const {
  ImageCoordinates c;
  c = undistort(imageX*imageWidth_, imageY*imageHeight_);
  Vector4d im{c(0), c(1), 1, 1};
  Vector4d target = camToWorld_ * im;
  Vector3d ray = (target.segment<3>(0) - cameraPosition_);
  float t = (height - cameraPosition_[2]) / ray[2];
  Vector3d w = ray * t + cameraPosition_;
  return WorldPosition(w[0], w[1], w[2]);
}

WorldPosition CameraMatrix::getWorldPosition(
    ImageCoordinates ic, float height) const {
  return getWorldPosition(ic[0], ic[1], height);
}

ImageCoordinates CameraMatrix::undistort(float x, float y) const {
  // TODO: implement
  return ImageCoordinates(x, y);
}

ImageCoordinates CameraMatrix::getImageCoordinates(
    float x, float y, float z) const {
  Vector4d v{x, y, z, 1};
  Vector4d c = worldToCam_ * v;
  c = c / c[2];
  return ImageCoordinates(c[0], c[1]);
}

ImageCoordinates CameraMatrix::getImageCoordinates(
    WorldPosition worldPosition) const {
  return getImageCoordinates(worldPosition[0],
                             worldPosition[1],
                             worldPosition[2]);
}

void CameraMatrix::setCalibration() {
  if (isTopCamera_) {
    fx_ = imageWidth_ / 2 / tan(FOVx / 2);
    fy_ = imageHeight_ / 2 / tan(FOVy / 2);
  } else {
    fx_ = imageWidth_ / 2 / tan(FOVx / 2);
    fy_ = imageHeight_ / 2 / tan(FOVy / 2);
  }
  scale_ = 0;
  cx_ = imageWidth_ / 2;
  cy_ = imageHeight_ / 2;
  cameraCalibration_ <<
    fx_, scale_,  cx_,
      0,    fy_,  cy_,
      0,      0,    1;
  coordinateShift_ <<
    0, -1,  0,
    0,  0, -1,
    1,  0,  0;
}

void CameraMatrix::updateCameraPose(const Pose3D& pose) {
  cameraRotation_ = pose.rotation.toRotationMatrix();

  for (int i = 0; i < 3; i++)
    cameraPosition_[i] = pose.translation[i];

  Eigen::Matrix3d R = coordinateShift_ * cameraRotation_.inverse();
  Vector3d t = R * -cameraPosition_;

  Eigen::Matrix4d K0 = Eigen::Matrix4d::Zero(), Rt = Eigen::Matrix4d::Zero();

  Rt.block<3, 3>(0, 0) = R;
  Rt.block<3, 1>(0, 3) = t;
  Rt(3, 3) = 1;

  K0.block<3, 3>(0, 0) = cameraCalibration_;
  K0(3, 3) = 1;

  worldToCam_ = K0 * Rt;
  camToWorld_ = worldToCam_.inverse();
}

float CameraMatrix::groundDistance(const WorldPosition& p) const {
  return sqrtf(p(0) * p(0) + p(1) * p(1));
}

float CameraMatrix::directDistance(const WorldPosition& p) const {
  Vector3d rel = p - cameraPosition_;
  return rel.norm();
}

float CameraMatrix::elevation(const WorldPosition& p) const {
  Vector3d rel = p - cameraPosition_;
  return acosf(rel[2] / rel.norm()) - M_PI / 2;
}

float CameraMatrix::bearing(const WorldPosition& p) const {
  return atan2(p(1), p(0));
}
