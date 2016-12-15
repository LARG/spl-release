#ifndef FIELD_EDGE_DETECTOR_H
#define FIELD_EDGE_DETECTOR_H

#include <memory/TextLogger.h>
#include <vision/Constants.h>
#include <common/Field.h>
#include <vision/BlobDetector.h>
#include <vision/ObjectDetector.h>
#include <vision/structures/BallCandidate.h>
#include <vision/structures/HorizonLine.h>
#include <vision/Macros.h>
#include <vision/estimators/BallEstimator.h>
#include <math/Point.h>
#include <math/Geometry.h>
#include <stdlib.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/// @ingroup vision
class FieldEdgeDetector : public ObjectDetector {
 public:
  FieldEdgeDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter, BlobDetector& blob_detector);


  bool detectFieldEdges();

 private:


  int *fieldEdgePoints;


  bool findFieldEdgePoints();
  bool ransacLine(int start, int end, Line2D &bestLine); 
  
  

//  Line2D fieldEdge


  int hstep, vstep;

  ColorSegmenter& color_segmenter_;
  BlobDetector& blob_detector_;



  cv::Mat m_img;


};

#endif
