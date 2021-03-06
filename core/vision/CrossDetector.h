#ifndef CROSS_DETECTOR_H
#define CROSS_DETECTOR_H

#include <vision/ObjectDetector.h>
#include <vision/ColorSegmenter.h>
#include <vision/BlobDetector.h>
#include <vision/Macros.h>
#include <vision/estimators/CrossEstimator.h>
#include <memory/TextLogger.h>
#include <algorithm>

/// @ingroup vision
class CrossDetector : public ObjectDetector {
  public:
    CrossDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter, BlobDetector& blob_detector);
    void detectCrosses();
    void init(TextLogger* tl){textlogger = tl;};

  private:
    void detectCross(BlobCollection& blobs);  // , WorldObject* cross);
    TextLogger* textlogger;

    ColorSegmenter& color_segmenter_;
    BlobDetector& blob_detector_;


    float checkGrayBelow(int xmin, int xmax, int ymin, int ymax, int hstep, int vstep);

    float getGreenPercentage(int,int,int,int,int,int);
    void correctRanges(int&,int&,int&,int&,int,int);
    float getDistanceByBlobWidth(float width);
    void setCrossObject(Blob& blob, WorldObject* cross);

    CrossEstimator estimator_;

};

#endif
