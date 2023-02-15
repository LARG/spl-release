#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <kinematics/ForwardKinematics.h>
#include <common/RobotDimensions.h>
#include <common/Profiling.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/VisionBlocks.h>
#include <common/RobotInfo.h>
#include <vision/structures/BallCandidate.h>
#include <math/Pose3D.h>
#include <vision/structures/VisionParams.h>
#include <vision/GoalDetector.h>
#include <vision/ROIDetector.h>
#include <vision/PenaltyKeeperImageProcessor.h>
#include <vision/RobotDetector.h>
#include <vision/YoloBallDetector.h>

class LineDetector;
class FieldEdgeDetector;
//class BallDetector;
class YoloBallDetector;
class BlobDetector;
class RobotDetector;
class CrossDetector;
class HoughDetector;
class ROIDetector;
class RamboGoalDetector;
class GoalDetector;
class HoughRobotDetector;
class ColorSegmenter;
class PenaltyKeeperImageProcessor;

/// @ingroup vision
class ImageProcessor {
  public:
   
    //typedef JerseyRobotDetector RobotDetector;
   
//    typedef HoughRobotDetector RobotDetector;
    
    /*typedef BandRobotDetector RobotDetector;*/
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera);
    ~ImageProcessor();
    void processFrame();
    void init(TextLogger*);
    void SetColorTable(unsigned char*);
    
    std::unique_ptr<FieldEdgeDetector> field_edge_detector_;
    std::unique_ptr<LineDetector> line_detector_;
    std::unique_ptr<RamboGoalDetector> rambo_goal_detector_;
    std::unique_ptr<GoalDetector> edge_goal_detector_;
    std::unique_ptr<YoloBallDetector> ball_detector_;
    std::unique_ptr<BlobDetector> blob_detector_;
    std::unique_ptr<RobotDetector> robot_detector_;
    std::unique_ptr<CrossDetector> cross_detector_;
    std::unique_ptr<ColorSegmenter> color_segmenter_;
    std::unique_ptr<HoughDetector> hough_detector_;
    std::unique_ptr<PenaltyKeeperImageProcessor> penalty_keeper_image_processor_;
    
    std::unique_ptr<ROIDetector> roi_detector_;
    unsigned char* getImg();
    unsigned char* getSegImg();
    unsigned char* getColorTable();
    bool isRawImageLoaded();
    int getImageHeight();
    int getImageWidth();
    const ImageParams& getImageParams() const { return iparams_; }
    const CameraMatrix& getCameraMatrix();
    void setCalibration(const RobotCalibration& calibration);
    void enableCalibration(bool value);
    void updateTransform();
    std::vector<BallCandidate> getBallCandidates();
    BallCandidate* getBestBallCandidate();
    bool isImageLoaded();
  private:
    int getTeamColor();
    double getCurrentTime();

    VisionBlocks& vblocks_;
    const ImageParams& iparams_;
    Camera::Type camera_;
    CameraMatrix cmatrix_;
    
    VisionParams vparams_;
    unsigned char* color_table_;
    TextLogger* textlogger;

    float getHeadPan() const;
    float getHeadTilt() const;
    float getHeadChange() const;
    
    std::unique_ptr<RobotCalibration> calibration_;
    bool enableCalibration_;

    void saveImg(std::string filepath);
    int topFrameCounter_ = 0;
    int bottomFrameCounter_ = 0;
};

#endif
