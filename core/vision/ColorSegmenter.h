#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <math/Point.h>
#include <list>
#include <vector>

#include <vision/Constants.h>
#include <vision/structures/VisionPoint.h>
#include <vision/structures/VisionParams.h>
#include <vision/structures/HorizonLine.h>
#include <vision/enums/Colors.h>
#include <vision/ColorTableMethods.h>
#include <vision/VisionBlocks.h>
#include <vision/structures/FocusArea.h>
#include <vision/structures/ROI.h>
#include <vision/Macros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <common/Profiling.h>
#include <vision/Logging.h>

/// @ingroup vision
class ColorSegmenter {
 public:
  ColorSegmenter(const VisionBlocks& vblocks, const VisionParams& vparams, const ImageParams& iparams, const Camera::Type& camera);
  ~ColorSegmenter();

  void init(TextLogger* tl){textlogger = tl;};

  VisionPoint ***horizontalPoint, ***verticalPoint;
  uint32_t **horizontalPointCount, **verticalPointCount;

  int *fieldEdgePoints;
  std::vector<cv::Point> field_edge_points;
  int *fieldEdgeRunLengths;

  bool classifyImage(unsigned char*);
  void classifyImage(const FocusArea& area);

  void constructRuns(int colorFlags = ~0);    // ~0 is a bit mask for all 1s -- ie. construct runs for all colors
  // Changing this does not save any time!!!
  // 0x71 -> 01110001 -> turns on green white orange robot white (~ 8ms)
  // 0x30 ->  00110000 -> turns on white orange (7-8 ms for other vision)
  // 0x70 -> 01110000 -> turns on green white orange (7-8 ms for other vision)
  // 0x7D -> 01111101 -> everything except undefined and yellow (11-13 ms)

  enum class Source {
    HighresGrayscale,
    Grayscale,
    Raw
  };
  inline cv::Mat extractMat(const ROI& roi, Source source = Source::Grayscale) const {
    switch(source) {
      case Source::Raw: return extractRawMat(roi);
      case Source::Grayscale: return extractGrayscaleMat(roi);
      case Source::HighresGrayscale: return extractHighresGrayscaleMat(roi);
      default: throw std::runtime_error("Invalid extraction source selected.");
    }
  }
  const cv::Mat& img_grayscale() const { return img_grayscale_; }
  cv::Mat& img_grayscale() { return img_grayscale_; }

  void setHorizon(HorizonLine);
  void preProcessPoints();
  void preProcessGoalPoints();
  bool didHighResBallScan = false;


  void setStepScale(int,int);
  void getStepSize(int&,int&) const;
  void getStepScale(int&,int&) const;
  bool startHighResGoalScan();
  bool startHighResBallScan();
  void completeHighResScan();
  inline Color xy2color(int x, int y) {
    return (Color)segImg_[y * iparams_.width + x];
  }

  float colorPercentageInBox(int xmin, int xmax, int ymin, int ymax, Color c);

 private:
  cv::Mat extractGrayscaleMat(const ROI& roi) const;
  cv::Mat extractRawMat(const ROI& roi) const;
  cv::Mat extractHighresGrayscaleMat(const ROI& roi) const;
  void clearPreviousHighResScans();
  void classifyImage(const std::vector<FocusArea>& areas, unsigned char*);
  void classifyImage(const FocusArea& area, unsigned char*);
  void constructRuns(const std::vector<FocusArea>& areas, int colorFlags);
  void constructRuns(const FocusArea& area, int colorFlags);

  bool startHighResScan(Color, int hStepScale = 0, int vStepScale = 0);
  void clearPoints(int colorFlags);
  bool prepareFocusAreas(std::vector<FocusArea>& areas, Color c);

  void completeHorizontalRun(uint8_t &hRunClr, int y, int hStartX, int finX);
  void completeVerticalRun(uint8_t &runClr, int x, int vStartY);

  bool setImagePointers();
  
  const VisionBlocks& vblocks_;
  const VisionParams& vparams_;
  const ImageParams& iparams_;
  const Camera::Type& camera_;
  bool initialized_;
  bool doingHighResScan_;
  bool bodyExclusionAvailable;
  TextLogger* textlogger;

  uint16_t* vGreenPosition;
  uint16_t* hGreenPosition;
  cv::Mat img_grayscale_;

  bool classifyingWholeImage;

  inline bool isBallPixel(uint8_t c){ return (c == c_WHITE || c == c_ROBOT_WHITE || c == c_UNDEFINED); }

  // For excluding parts of the shoulder in the body
  float bodyExclusionSlope[NUM_BODY_EXCL_POINTS];
  float bodyExclusionOffset[NUM_BODY_EXCL_POINTS];

  unsigned char* img_;
  unsigned char* segImg_, *segImgLocal_;
  uint16_t vstep_, hstep_, vscale_, hscale_;
  HorizonLine horizon_;
  unsigned char* colorTable_;
  bool* pointScanned;
  bool fromLog_;
  bool fillGreenPct_;
};
#endif
