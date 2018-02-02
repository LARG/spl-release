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
#include <stack>
#include <vector>

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
  std::vector<LineSegment> hullSegments;

 private:

  bool removeOutliers();
  FieldEdgePoint nextToTop(stack<FieldEdgePoint> &S);
  bool findFieldEdgePoints();
  int orientation(FieldEdgePoint p, FieldEdgePoint q, FieldEdgePoint r);
  void convexHull();

  ColorSegmenter& color_segmenter_;
  BlobDetector& blob_detector_;

};

#endif
