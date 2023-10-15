#ifndef Pose3D_H
#define Pose3D_H

#include <Eigen/Dense>
#include <iostream>

/** representation for 3D Transformation (Location + Orientation)*/
class Pose3D {
 public:
  Eigen::Quaterniond rotation;
  Eigen::Vector3d translation;

  /** constructor*/
  Pose3D() : rotation(1, 0, 0, 0), translation(0, 0, 0) {}

  /** constructor from rotation and translation
   * \param rot Rotation
   * \param trans Translation
   */
  Pose3D(const Eigen::Quaterniond& rot, const Eigen::Vector3d& trans):
      rotation(rot), translation(trans) {}

  /** constructor from rotation
   * \param rot Rotation
   */
  explicit Pose3D(const Eigen::Quaterniond& rot):
    rotation(rot), translation(0, 0, 0) {}

  /** constructor from translation
   * \param trans Translation
   */
  explicit Pose3D(const Eigen::Vector3d& trans):
    rotation(1, 0, 0, 0), translation(trans) {}

  /** constructor from translation and euler rotation
    * \param trans Translation
    * \param rotE Euler rotation
    */
  explicit Pose3D(const Eigen::Vector3d & trans, const Eigen::Vector3d& rotE):
    translation(trans) {
    Eigen::AngleAxisd rollAngle(rotE[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rotE[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rotE[2], Eigen::Vector3d::UnitZ());
    rotation = yawAngle * pitchAngle * rollAngle;
  }

  /** constructor from three translation values
   * \param x translation x component
   * \param y translation y component
   * \param z translation z component
   */
  Pose3D(const float x, const float y, const float z) :
    rotation(1, 0, 0, 0), translation(x, y, z) {}

  /** Assignment operator
  *\param other The other Pose3D that is assigned to this one
  *\return A reference to this object after the assignment.
  */
  Pose3D& operator=(const Pose3D& other) {
    rotation = other.rotation;
    translation = other.translation;
    return *this;
  }

  /** Copy constructor
  *\param other The other vector that is copied to this one
  */
  Pose3D(const Pose3D& other) {*this = other;}

  /** Multiplication with Point
  *\param point (Vector3&lt;float&gt;)
  */
  Eigen::Vector3d operator*(const Eigen::Vector3d& point) const {
    return rotation * point + translation;
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are equal.
  */
  bool operator==(const Pose3D& other) const {
    return (translation == other.translation) &&
        (rotation.isApprox(other.rotation));
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are unequal.
  */
  bool operator!=(const Pose3D& other) const {
    return !(*this == other);
  }

  /**Concatenation of this pose with another pose
  *\param other The other pose that will be concatenated to this one.
  *\return A reference to this pose after concatenation
  */
  Pose3D& conc(const Pose3D& other) {
    translation = *this * other.translation;
    rotation *= other.rotation;
    return *this;
  }

  Pose3D relativeTo(const Pose3D& other) const {
    Pose3D result;
    result.rotation = other.rotation.conjugate() * this->rotation;
    result.translation = other.rotation.conjugate() *
      (-other.translation + this->translation);
    return result;
  }

  /** Calculates the inverse transformation from the current pose
  * @return The inverse transformation pose.
  */
  Pose3D invert() const {
    Pose3D result;
    result.rotation = this->rotation.conjugate();
    result.translation = result.rotation *
      (Eigen::Vector3d(0, 0, 0) - this->translation);
    return result;
  }

  /**Translate this pose by a translation vector
  *\param trans Vector to translate with
  *\return A reference to this pose after translation
  */
  Pose3D& translate(const Eigen::Vector3d& trans) {
    translation = *this * trans;
    return *this;
  }

  /**Translate this pose by a translation vector
  *\param x x component of vector to translate with
  *\param y y component of vector to translate with
  *\param z z component of vector to translate with
  *\return A reference to this pose after translation
  */
  Pose3D& translate(const float x, const float y, const float z) {
    Eigen::Vector3d vec(x, y, z);
    return translate(vec);
  }

  /**Rotate this pose by a rotation
  *\param rot Rotationmatrix to rotate with
  *\return A reference to this pose after rotation
  */
  Pose3D& rotate(const Eigen::Quaterniond& rot) {
    rotation = rot*rotation;
    return *this;
  }

  /**Rotate this pose around its x-axis
  *\param angle angle to rotate with
  *\return A reference to this pose after rotation
  */
  Pose3D& rotateX(const float angle) {
    return this->rotate(Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, Eigen::Vector3d(1, 0, 0))));
  }

  /**Rotate this pose around its y-axis
  *\param angle angle to rotate with
  *\return A reference to this pose after rotation
  */
  Pose3D& rotateY(const float angle) {
    return this->rotate(Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 1, 0))));
  }

  /**Rotate this pose around its z-axis
  *\param angle angle to rotate with
  *\return A reference to this pose after rotation
  */
  Pose3D& rotateZ(const float angle) {
    return this->rotate(Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 0, 1))));
  }

  Pose3D relativeToGlobal(const Pose3D &origin) {
    Pose3D retval(*this);
    retval.translation = origin.rotation * retval.translation;
    retval.translation += origin.translation;
    retval.rotation = origin.rotation * retval.rotation;
    return retval;
  }

/**  Pose3D globalToRelative(const Pose3D &origin){
    Pose2D origin_2d(0,0,0);
    origin_2d.translation.x = origin.translation.x;
    origin_2d.translation.y = origin.translation.y;
    origin_2d.rotation = origin.rotation.getZAngle();
    return this->globalToRelative(origin_2d);
  }*/
};

#endif  // Pose3D_H
