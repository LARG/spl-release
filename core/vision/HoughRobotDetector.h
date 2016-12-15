#ifndef HOUGH_ROBOTDETECTOR_H
#define HOUGH_ROBOTDETECTOR_H

#include <vision/ObjectDetector.h>
#include <memory/TextLogger.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/structures/Blob.h>
#include <vision/enums/Colors.h>
#include <vision/Macros.h>
#include <vision/BlobDetector.h>
#include <vision/HoughDetector.h>
#include <vision/ColorSegmenter.h>
#include <vision/structures/RobotCandidate.h>
#include <vision/estimators/RobotEstimator.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>


/// @ingroup vision
class HoughRobotDetector : public ObjectDetector {
 public:
  HoughRobotDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter, BlobDetector& blob_detector, HoughDetector& hough_detector);


  void init(TextLogger* tl){textlogger = tl;};

  
  void detectRobots(); 
  
 
  // Old stuff
  void detectBlueRobots();
  void detectPinkRobots();
  std::list<Blob*> getBlueRobots();
  std::list<Blob*> getPinkRobots();
 
 
 
 private:
  void detectRobots(Color c);
  std::vector<RobotCandidate> formRobotCandidates(std::vector<Blob*>& blobs, Color c);
  void computeCandidateProbabilities(std::vector<RobotCandidate>& candidates);
  void selectRobots(std::vector<RobotCandidate>& candidates);
  float getDistanceByWidth(float width);
  float getDistanceByHeight(float height);
  void fillColorPercents(RobotCandidate& candidate);
  void fillFeet(RobotCandidate& candidate);
  ColorSegmenter& color_segmenter_;
  BlobDetector& blob_detector_;
  HoughDetector& hough_detector_;
  TextLogger* textlogger;
  std::list<Blob*> blueRobots_, pinkRobots_;
  RobotEstimator estimator_;

  int hstep, vstep;

  int expandRobot(const Blob &bi, const Blob &bj);

  cv::Mat goalPostImage;

};
#endif
