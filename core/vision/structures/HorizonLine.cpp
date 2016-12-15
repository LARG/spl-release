#include <vision/structures/HorizonLine.h>

HorizonLine HorizonLine::generate(const ImageParams& iparams, const CameraMatrix& camera, int distance) {
  // The horizon is the line drawn on the ground directly in front of the camera at a distance of X meters at the left/right edges
  Position middle = camera.getWorldPosition(iparams.width / 2, iparams.height - 1);
  float bearingOffset = atan2(middle.y, middle.x);

  float bearingLeft = FOVx / 2 + bearingOffset;
  float bearingRight = -FOVx / 2 + bearingOffset;

  float lx = distance * cosf(bearingLeft);
  float ly = distance * sinf(bearingLeft);
  float rx = distance * cosf(bearingRight);
  float ry = distance * sinf(bearingRight);

  HorizonLine horizon;

  auto left = camera.getImageCoordinates(lx, ly, 0);
  auto right = camera.getImageCoordinates(rx, ry, 0);

  horizon.gradient = (float)(right.y - left.y) / (right.x - left.x);
  if(left.x != right.x)
    horizon.offset = left.y - horizon.gradient * left.x;
  else
    horizon.offset = 0;
  horizon.exists = true;

  return horizon;
}
