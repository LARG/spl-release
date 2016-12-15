#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <common/RobotInfo.h>
#include <vision/structures/VisionParams.h>
#include <vision/CameraMatrix.h>
#include <vision/VisionBlocks.h>
#include <common/Profiling.h>
#include <memory/TextLogger.h>

#define DETECTOR_DECLARE_ARGS VisionBlocks& vblocks, const VisionParams& vparams, const ImageParams& iparams, const CameraMatrix& cmatrix, const Camera::Type& camera
#define DETECTOR_INITIALIZE ObjectDetector(vblocks, vparams, iparams, cmatrix, camera)
#define DETECTOR_PASS_ARGS vblocks_, vparams_, iparams_, cmatrix_, camera_

/// @ingroup vision
class ObjectDetector {
  protected:
    VisionBlocks& vblocks_;
    const VisionParams& vparams_;
    const ImageParams& iparams_;
    const CameraMatrix& cmatrix_;
    const Camera::Type& camera_;
    TextLogger* textlogger;
    int processed_frames_ = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ObjectDetector(DETECTOR_DECLARE_ARGS) :
      vblocks_(vblocks), vparams_(vparams), iparams_(iparams), cmatrix_(cmatrix), camera_(camera) { }
    virtual void init(TextLogger* tl){textlogger = tl;};
    virtual void processFrame();

};
#endif
