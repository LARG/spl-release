#ifndef FIELD_EDGE_DETECTOR_H
#define FIELD_EDGE_DETECTOR_H

#include <memory/TextLogger.h>
#include <vision/Constants.h>
#include <common/Field.h>
#include <vision/BlobDetector.h>
#include <vision/ObjectDetector.h>
#include <vision/structures/BallCandidate.h>
#include <vision/structures/HorizonLine.h>
#include <vision/structures/VisionObjectCandidate.h>
#include <vision/Macros.h>
#include <vision/estimators/BallEstimator.h>
#include <math/Point.h>
#include <math/Geometry.h>
#include <stdlib.h>
#include <time.h>
#include <stack>
#include <vector>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

/// @ingroup vision



class FieldEdgePoint : public Point2D {
  public:
//  int x;
//  int y;
  bool valid;
  bool below;
  int hullY;
  FieldEdgePoint(int X, int Y): Point2D(X, Y), valid(true), below(false) {}
};






class FieldEdgeDetector : public ObjectDetector {
 public:
  FieldEdgeDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter, BlobDetector& blob_detector);


  bool detectFieldEdges();


  int *fieldEdgePoints;


  int hstep, vstep;

  std::vector<FieldEdgePoint> hullPoints;
  std::vector<FieldEdgePoint> hullPointCands;

  std::vector<VisionObjectCandidate> objects;


  std::vector<LineSegment> hullSegments;

 private:

  bool removeOutliers();

  void filterObjects();

  FieldEdgePoint nextToTop(stack<FieldEdgePoint> &S);

  bool ransacLine(int start, int end, Line2D &bestLine); 
  
  bool findFieldEdgePoints();
  
  int orientation(FieldEdgePoint p, FieldEdgePoint q, FieldEdgePoint r);

//  Line2D fieldEdge

  bool continueObjectRun(FieldEdgePoint &p, int lastY);

  void findPossibleObjects();
  void convexHull();

  ColorSegmenter& color_segmenter_;
  BlobDetector& blob_detector_;


  void sortObjects();

//  cv::Mat m_img;


};

#endif
