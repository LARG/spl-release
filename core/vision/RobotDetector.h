#ifndef ROBOTDETECTOR_H
#define ROBOTDETECTOR_H

#include <vision/ObjectDetector.h>
#include <memory/TextLogger.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/structures/Blob.h>
#include <vision/enums/Colors.h>
#include <vision/Macros.h>
#include <vision/BlobDetector.h>
#include <vision/ColorSegmenter.h>
#include <vision/structures/RobotCandidate.h>
#include <vision/estimators/RobotEstimator.h>
#include <vision/FieldEdgeDetector.h>
#include <vision/structures/ObjectCandidate.h>
#include <common/RelayStruct.h>

/// @ingroup vision
class RobotDetector : public ObjectDetector {
 public:
  RobotDetector(DETECTOR_DECLARE_ARGS, FieldEdgeDetector& field_edge_detector, ColorSegmenter& segmenter);
  void init(TextLogger* tl){textlogger = tl;};
  
 
  
  void detectRobots();


  void selectRobots(std::vector<RobotCandidate>& candidates);


  FieldEdgeDetector& field_edge_detector_;
  ColorSegmenter& color_segmenter_;






// ============= //  
  
  void detectBlueRobots();
  void detectPinkRobots();
  void detectRobotCluster();
  std::list<Blob*> getBlueRobots();
  std::list<Blob*> getPinkRobots();

// ============= //

 private:

// ============= //

  void detectRobots(Color c);
  std::vector<RobotCandidate> formRobotCandidates(std::vector<Blob*>& blobs, Color c);
  void computeCandidateProbabilities(std::vector<RobotCandidate>& candidates);
  float getDistanceByWidth(float width);
  float getDistanceByHeight(float height);
  void fillColorPercents(RobotCandidate& candidate);
  void fillFeet(RobotCandidate& candidate);
  
// ============= //


//  ColorSegmenter*& color_segmenter_;
//  BlobDetector*& blob_detector_;
  TextLogger* textlogger;
  std::list<Blob*> blueRobots_, pinkRobots_;
  RobotEstimator estimator_;


};
#endif
