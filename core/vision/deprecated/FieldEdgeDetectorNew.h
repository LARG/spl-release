#ifndef FIELD_EDGE_DETECTOR_H
#define FIELD_EDGE_DETECTOR_H

#include <memory/TextLogger.h>
#include <constants/VisionConstants.h>
#include <common/Field.h>
#include <common/Profiling.h>
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
    FieldEdgeDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector);


    bool detectFieldEdges();

  private:


    int *consensusCF;   // Cumulative function for consensus points. Note there is a break between left and right regions where the function resets. 


    int VOTE_THRESH;
    const float CONSENSUS_PCT = 0.6;


    bool ransacLine(int start, int end, Line2D &bestLine); 

    void findPointsBelowLine(Line2D line);


    LineSegment truncateLine(Line2D &line, int start, int end);

    bool isOneLine(Line2D& lineA, Line2D& lineB);


    bool projectAndFillEdge(LineSegment &lineSeg);
    void setFieldEdgeObject(int woIndex, const LineSegment &edge);

    int hstep, vstep;

    Classifier*& classifier_;
    BlobDetector*& blob_detector_;


    LineSegment fieldEdges[2];       // Field edges in world space
    int fieldEdgeCounter;


    Timer timer1, timer2, timer3;

    cv::Mat m_img;


};

#endif
