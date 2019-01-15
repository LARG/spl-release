#pragma once

#include <common/Profiling.h>
#include <memory/TextLogger.h>
#include <vision/ColorSegmenter.h>
#include <vision/FieldEdgeDetector.h>
#include <vision/ObjectDetector.h>
#include <vision/structures/ROI.h>
#include <vision/structures/HorizonLine.h>
#include <math/Geometry.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct ROIRect {
  int xmin, xmax, ymin, ymax;
  int area() const { return (xmax - xmin) * (ymax - ymin); }
  int boxRow, boxCol;
  vector<cv::Vec4i> lines;
  bool valid;
  vector<cv::Vec4i> lineCandidate;
  LineSegment avgLine;
  ROIRect() : valid(false) { }
};

struct LineStack {
  std::array<uint16_t,50> lineIndices;
  uint16_t lineCount;
};

/// @ingroup vision
class ROIDetector : public ObjectDetector {
  public:
    ROIDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter, FieldEdgeDetector& field_edge_detector);
    void init(TextLogger* tl){textlogger = tl;};
    std::vector<ROI> findROIs();
    void findBallROIs();
    inline const int xstep() const { return xstep_; }
    inline const int ystep() const { return ystep_; }
    inline const int scale() const { return scale_; }
    cv::Mat extractMat(const ROI& roi, bool highres=false) const;
    cv::Mat extractMat(const ROI& roi, int scaleFactor) const;
    void setHorizon(const HorizonLine& horizon) { horizon_ = horizon; }
     
    cv::Mat binaryImg;
    std::vector<int> seams;
    std::vector<cv::Point> points;
    std::vector<ROI> ballROIs;

    // Accessors for tool
    vector<vector<ROIRect>> toolBoxMap; 
    cv::Mat edgeImg;

    // for debugging
    int frame_counter_ = 1;
    int pixel_counter_ = 0;
    int max_pixel_count_ = 0;
    void drawROIs(std::vector<ROI>& rois, int xstep, int ystep);
    int nROIPoints = 0;
    int nROIRegions = 0;

  private:
    TextLogger* textlogger;
    HorizonLine horizon_;
    void extractMat(cv::Mat &img) const;
    ColorSegmenter& color_segmenter_;
    FieldEdgeDetector& field_edge_detector_;
    unsigned char *img_;
    int xstep_, ystep_;  // how to subsample the raw image
    const int scale_ = 16;  // number of pixels (in subsampled image) per box
    const int horizonY = 0;
    cv::Mat field_line_img_;

    bool setImagePointers();
    std::vector<cv::Vec4i> findLinesInBox(vector<cv::Vec4i> lines);
    void mergeBoxes(ROIRect &source, ROIRect &target);
    std::vector<LineStack> constructLines(vector<ROIRect> &allLineCands);
    cv::Vec4i averageLines(cv::Vec4i line1, cv::Vec4i line2);
    void recurseCheck(vector<ROIRect> &allLineCands, LineStack &linestack, int i);
    int chooseThreshold(int window, int band_index);
    float chooseWindowFactor(int band_index);
  
    void padRoisTopCamera(std::vector<ROI>& rois);
    void padRoisBottomCamera(std::vector<ROI>& rois);

    float checkGreenBelowPct(const ROI &roi);
    float checkGreenInsidePct(const ROI &roi);

};
