#pragma once

#include <vision/Constants.h>
#include <common/Field.h>
#include <vision/ObjectDetector.h>
#include <vision/FieldEdgeDetector.h>
#include <vision/structures/BallCandidate.h>
#include <vision/structures/RobotCandidate.h>
#include <vision/structures/HorizonLine.h>
#include <vision/structures/ROI.h>
#include <vision/Macros.h>
#include <math/Point.h>
#include <common/Profiling.h>
#include <common/Roles.h>
#include <common/ColorConversion.h>
#include <vision/BlobDetector.h>
// #include <vision/estimators/MovingBallEstimator.h>
// #include <vision/estimators/PenaltyBallEstimator.h>

// Adding ALL the files that Mauricio and Sai added in their cpp
// implementation
// TODO: Sai/Mauricio remove unneeded ones
#include <fcntl.h>      // NOLINT(build/include_order)
#include <getopt.h>     // NOLINT(build/include_order)
#include <sys/time.h>   // NOLINT(build/include_order)
#include <sys/types.h>  // NOLINT(build/include_order)
#include <sys/uio.h>    // NOLINT(build/include_order)
#include <unistd.h>     // NOLINT(build/include_order)

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

// If compiling tool we have to comment the opencv files below
#ifdef TOOL
#include <opencv2/core/core.hpp>
#else
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#endif
#include <detect_lib.h>

// using namespace cv; // Is causing issues in tool compilation
using namespace std;
using CrossCandidate = BallCandidate;

class Classifier;
// class ROIDetector;

/// @ingroup vision
class YoloBallDetector : public ObjectDetector {
 public:
  YoloBallDetector(DETECTOR_DECLARE_ARGS, FieldEdgeDetector& field_edge_detector);
  ~YoloBallDetector();
  void init(TextLogger* tl) final;
  void findBall();
  inline void setHorizon(HorizonLine horizon) { horizon_ = horizon; }
  inline std::vector<BallCandidate> candidates() const { return candidates_; }
  inline std::vector<BallCandidate> cross_candidates() const { return candidates_; }
  inline BallCandidate* best() const { return best_.get(); }
  cv::Mat blobDebugImg;
  bool setImagePointers();

 private:
  // values  used by the network
  const int width_top = 224;
  const int height_top = 120;
  const int width_bottom = 80;
  const int height_bottom = 80;
  const int num_output_boxes = 420;
  const int num_classes = 1;
  const float min_confidence = 0.65;
  
  float getDirectDistanceByBlobWidth(float width, int centerX, int centerY);
  float getDirectDistanceByKinematics(int x, int y);
  float getDirectDistance(BallCandidate& candidate);
  bool setBest(BallCandidate& candidate);
  bool setCrossBest(CrossCandidate& candidate);
  bool findMovingBall();
  void fitCircleToPoints(uint16_t*, uint16_t*, uint16_t, float*, float*, float*, float*);

  void selectRobots(std::vector<RobotCandidate>& candidates);

  //BlobTriangle blobTests(cv::Mat& mat, const ROI& roi, int xstep, int ystep, bool highres);
  //BlobTriangle blobGeometryTests(std::vector<BlobTriangle>& triangles, const std::vector<BallBlob>& blobs, const std::vector<BallBlob>& badBlobs, const ROI& roi, int xstep, int ystep);
  //BlobTriangle blobGeometryTests(BlobTriangle& triangle, const std::vector<BallBlob>& blobs, const std::vector<BallBlob>& badBlobs, const ROI& roi, int xstep, int ystep);


  // for debugging
  int counter_ = 0;

  float max(float a, float b, float c);
  float min(float a, float b, float c);

  std::vector<BallCandidate> candidates_;
  std::vector<BallCandidate> cross_candidates_;
  std::unique_ptr<BallCandidate> best_;
  std::unique_ptr<BallCandidate> cross_best_;
  
  std::vector<RobotCandidate> robot_candidates_;
  // std::unique_ptr<RobotCandidate> robot_best_;
  
  std::unique_ptr<Classifier> classifier_;
  HorizonLine horizon_;
  
  unsigned char *img_;
  FieldEdgeDetector& field_edge_detector_;

  TFLiteDetector *detector_;
  TFLiteDetector *bottom_detector_;
  TFLiteDetector *top_detector_;

  DetectLibConfig * top_detector_config;
  DetectLibConfig * bottom_detector_config;
};
