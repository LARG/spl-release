#ifndef CAMERA_MATRIX_H
#define CAMERA_MATRIX_H

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#define FOVx (56.3 / 180.0 * M_PI)
#define FOVy (43.7 / 180.0 * M_PI)

#include <Eigen/Dense>
#include <Eigen/LU>
#include <string>
#include "localization/types.h"
#include "localization/RobotDimensions.h"
#include "localization/Pose3D.h"

class CameraMatrix {
 private:
    Eigen::Matrix4d worldToCam_, camToWorld_;
    Eigen::Matrix3d coordinateShift_;
    Eigen::Matrix3d cameraCalibration_;
    Eigen::Vector3d cameraPosition_;
    Eigen::Matrix3d cameraRotation_;
    const int imageWidth_;
    const int imageHeight_;
    const bool isTopCamera_;
    float fx_, fy_, scale_, cx_, cy_;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraMatrix(
        const bool isTopCamera = true,
        const int imageWidth = IMAGE_WIDTH,
        const int imageHeight = IMAGE_HEIGHT);

    inline float getCameraBearing(int imageX) const {
      return atan((imageWidth_/2 - imageX) /
          (imageWidth_/(2 * tan(FOVx/2))));
    }
    inline float getCameraElevation(int imageY) const {
      return atan((imageHeight_/2 - imageY) /
          (imageHeight_/(2 * tan(FOVy/2))));
    }

    inline float getImageX(float cameraBearing) const {
      return (imageWidth_ / 2) * (1 - (tan(cameraBearing) / tan(FOVx / 2)));
    }
    inline float getImageY(float cameraElevation) const {
      return (imageHeight_ / 2) * (1 - (tan(cameraElevation) / tan(FOVy / 2)));
    }

    inline int getImageWidth() const { return imageWidth_; }
    inline int getImageHeight() const { return imageHeight_; }

    WorldPosition getWorldPosition(
        float imageX, float imageY, float height = 0.0f) const;
    WorldPosition getWorldPosition(
        ImageCoordinates ic, float height = 0.0f) const;

    ImageCoordinates undistort(float x, float y) const;
    ImageCoordinates getImageCoordinates(float x, float y, float z) const;
    ImageCoordinates getImageCoordinates(WorldPosition worldPosition) const;

    void updateCameraPose(const Pose3D& pose);

    float groundDistance(const WorldPosition& p) const;
    float directDistance(const WorldPosition& p) const;
    float elevation(const WorldPosition& p) const;
    float bearing(const WorldPosition& p) const;

    void setCalibration();
};

#endif
