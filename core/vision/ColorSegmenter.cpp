#include <vision/ColorSegmenter.h>
#include <yuview/YUVImage.h>

#include <fstream>
#include <iostream>
#include <common/ColorSpaces.h>

#define SEGMENT_ABOVE_HORIZON true

using namespace cv;

ColorSegmenter::ColorSegmenter(const VisionBlocks& vblocks, const VisionParams& vparams, const ImageParams& iparams, const Camera::Type& camera) :
    vblocks_(vblocks), vparams_(vparams), iparams_(iparams), camera_(camera), initialized_(false), doingHighResScan_(false) {

  segImg_ = new unsigned char[iparams.size];
  segImgLocal_ = segImg_;
  vGreenPosition = new uint16_t[iparams_.width];
  hGreenPosition = new uint16_t[iparams_.height];

  horizontalPoint = new VisionPoint**[Color::NUM_Colors];
  for(int i=0;i<Color::NUM_Colors;i++) {
    horizontalPoint[i] = new VisionPoint*[iparams_.height];
    for(int j = 0; j < iparams_.height; j++)
      horizontalPoint[i][j] = new VisionPoint[iparams_.width];
  }
  verticalPoint = new VisionPoint**[Color::NUM_Colors];
  for(int i=0;i<Color::NUM_Colors;i++) {
    verticalPoint[i] = new VisionPoint*[iparams_.width];
    for(int j = 0; j < iparams_.width; j++)
      verticalPoint[i][j] = new VisionPoint[iparams_.height];
  }
  horizontalPointCount = new uint32_t*[Color::NUM_Colors];
  for(int i=0;i<Color::NUM_Colors;i++) {
    horizontalPointCount[i] = new uint32_t[iparams_.height];
  }
  verticalPointCount = new uint32_t*[Color::NUM_Colors];
  for(int i=0;i<Color::NUM_Colors;i++)
    verticalPointCount[i] = new uint32_t[iparams_.width];

  bodyExclusionAvailable = false;
  fillGreenPct_ = false;

  setStepScale(iparams_.defaultHorizontalStepScale, iparams_.defaultVerticalStepScale);
  setImagePointers();

  img_grayscale_ = Mat::zeros(iparams_.height, iparams_.width, CV_8UC1);

  classifyingWholeImage = false;


 fieldEdgePoints = new int[iparams_.width / (1 << iparams_.defaultHorizontalStepScale)](); 
 fieldEdgeRunLengths = new int[iparams_.width / (1 << iparams_.defaultHorizontalStepScale)]();
}

ColorSegmenter::~ColorSegmenter() {
  delete [] vGreenPosition;
  delete [] hGreenPosition;

  for(int i=0;i<Color::NUM_Colors;i++) {
    for(int j = 0; j < iparams_.height; j++)
      delete [] horizontalPoint[i][j];
    delete [] horizontalPoint[i];
  }
  delete [] horizontalPoint;

  for(int i=0;i<Color::NUM_Colors;i++) {
    for(int j = 0; j < iparams_.width; j++)
      delete [] verticalPoint[i][j];
    delete [] verticalPoint[i];
  }
  delete [] verticalPoint;
  
  for(int i=0;i<Color::NUM_Colors;i++)
    delete [] horizontalPointCount[i];
  delete [] horizontalPointCount;
  
  for(int i=0;i<Color::NUM_Colors;i++)
    delete [] verticalPointCount[i];
  delete [] verticalPointCount;
  delete [] segImgLocal_;
}

bool ColorSegmenter::setImagePointers() {
  if(vblocks_.image == NULL) {
    printf("No image block loaded! Classification failed.\n");
    return false;
  }
  if(vblocks_.robot_vision == NULL) {
    printf("No vision block loaded! Classification failed.\n");
    return false;
  }
  bool imageLoaded = vblocks_.image->isLoaded();
  if(camera_ == Camera::TOP) {
    #ifdef TOOL
    if(imageLoaded) {
    #endif
      vblocks_.robot_vision->setSegImgTop(segImg_);
      img_ = vblocks_.image->getImgTop();
      if(!img_) return false;
    #ifdef TOOL
    } else if(vblocks_.robot_vision->isLoaded()) {
      segImg_ = vblocks_.robot_vision->getSegImgTop();
    }
    #endif
  }
  else {
    #ifdef TOOL
    if(imageLoaded) {
    #endif
      vblocks_.robot_vision->setSegImgBottom(segImg_);
      img_ = vblocks_.image->getImgBottom();
      if(!img_) return false;
    #ifdef TOOL
    } else if(vblocks_.robot_vision->isLoaded()) {
      segImg_ = vblocks_.robot_vision->getSegImgBottom();
    }
    #endif
  }
  if(!initialized_) {
    #ifdef TOOL
    if(imageLoaded)
    #endif
    memset(segImg_, c_UNDEFINED, sizeof(unsigned char) * iparams_.size);
    initialized_ = true;
  }
  return true;
}

bool ColorSegmenter::classifyImage(unsigned char *colorTable) {
  if(!setImagePointers()) {
    return false;
  } 
  clearPreviousHighResScans();
  FocusArea area(0, 0, iparams_.width - 1, iparams_.height - 1);
  
 
  if (vblocks_.game_state->state() == INITIAL){
    fillGreenPct_ = true;
  }
  
  // TODO: SANMIT: This needs to match the sampling rate for the goal detection code
  //if (vblocks_.frame_info->frame_id % 3 == 0){
    for (int i = 0; i < iparams_.width / hstep_; i++){
      fieldEdgeRunLengths[i] = 0; 
    }

    classifyingWholeImage = true;
  //}
 
  VisionTimer::Start(20, "Segmenter(%s)::classifyImage", camera_);
  classifyImage(area, colorTable);
  VisionTimer::Stop("Segmenter(%s)::classifyImage", camera_);
  classifyingWholeImage = false;
  fillGreenPct_ = false;
  return true;
}

void ColorSegmenter::classifyImage(const std::vector<FocusArea>& areas, unsigned char *colorTable) {
  if(!setImagePointers()) return;
  for(unsigned int i = 0; i < areas.size(); i++)
    classifyImage(areas[i], colorTable);
}

void ColorSegmenter::classifyImage(const FocusArea& area) {
  if(!setImagePointers()) return;
  classifyImage(area, colorTable_);
}

void ColorSegmenter::classifyImage(const FocusArea& area, unsigned char* colorTable){
  bool imageLoaded = vblocks_.image->isLoaded();
  if(!imageLoaded) {
    tlog(20, "Classifying with no raw image");
  }
  if(fillGreenPct_) {
    if(camera_ == Camera::TOP) {
      vblocks_.robot_vision->topGreenPct = 0;
      vblocks_.robot_vision->topUndefPct = 0;
    }
    else {
      vblocks_.robot_vision->bottomGreenPct = 0;
      vblocks_.robot_vision->bottomUndefPct = 0;
    }
  }
/*
  if (vblocks_.robot_state->WO_SELF == WO_TEAM_COACH){
    hstep_=1;
    vstep_=1;
  }
*/

  colorTable_ = colorTable;
  tlog(28, "Classifying on area %i,%i to %i,%i with horizon %2.f,%2.f and step sizes H %i V %i", area.x1, area.y1, area.x2, area.y2, horizon_.gradient, horizon_.offset, hstep_, vstep_);
  int pixCount = 0;

  int id = tic();


#ifdef TOOL

//  ofstream greenFile, whiteFile, grayFile, blackFile;
//  greenFile.open("/home/sanmit/Desktop/histoData/green.txt", ios::out | ios::app);
//  whiteFile.open("/home/sanmit/Desktop/histoData/white.txt", ios::out | ios::app);
//  grayFile.open("/home/sanmit/Desktop/histoData/gray.txt", ios::out | ios::app); 
//  blackFile.open("/home/sanmit/Desktop/histoData/black.txt", ios::out | ios::app);

#endif 

  field_edge_points.clear();
  int prev_y = -1;
  for (int y = area.y1; y <= area.y2; y += vstep_) {
    if(!SEGMENT_ABOVE_HORIZON) {
      int midX = (area.x1 + area.x2) / 2;
      if(!horizon_.isAbovePoint(midX, y)) continue;
    }
    for(int x = area.x1; x <= area.x2; x += hstep_) {
      pixCount++;
      Color c;
#ifdef TOOL
      if (imageLoaded) // if a raw image is available
#endif
      {

        int gray;
        //c = ColorTableMethods::xy2color(img_, colorTable, x, y, iparams_.width);
        c = ColorTableMethods::xy2color(img_, colorTable, x, y, iparams_.width, gray);
        img_grayscale_.at<unsigned char>(y,x) = gray;


#ifdef TOOL        
//        // Save out by color
//        int yy,uu,vv;
//        ColorTableMethods::xy2yuv(img_, x, y, iparams_.width, yy, uu, vv);
//
//
//
//        // Experimental: change c to use some simple thresholds on color histogram
//        if (uu <= 140 && uu >= 110 && vv <= 140 && vv >= 110){
//          if (yy >= 150)
//            c = c_WHITE;
//          else
//            c = c_BLUE;
//        }
//        else if (uu >= 80 && uu <= 140 && vv >= 70 && vv <= 110 && yy >= 50 && yy <= 175){
//          c = c_FIELD_GREEN;
//        }
//        else {
//          c = c_UNDEFINED;
//        }
//  






//        RGB rgb = YUV444_TO_RGB(yy, uu, vv);
//        yy = rgb.r;
//        uu = rgb.g;
//        vv = rgb.b;
//
//        if (c == c_FIELD_GREEN){
//          greenFile << yy << "," << uu << "," << vv << "\n";
//        }
//        else if (c == c_WHITE){
//          whiteFile << yy << "," << uu << "," << vv << "\n";
//        }
//        else if (c == c_ROBOT_WHITE){
//          grayFile << yy << "," << uu << "," << vv << "\n";
//        }
//        else if (c == c_BLUE){
//          blackFile << yy << "," << uu << "," << vv << "\n";
//        }



#endif


     /* Everything commented out below here is really slow. I hope it's not necessary! - JM 04/20/2016 */
        
     /*
      if (classifyingWholeImage){        

        if (c==c_FIELD_GREEN){
          if (fieldEdgeRunLengths[x/hstep_] == 0 && x % 20 == 0){
            //fieldEdgePoints[x/hstep_] = y;
            // iparams_.height - y because convex hull assumes origin is at bottom
            // make sure to do iparams_.height - y afterwards to get original y value

            // y-value should not be too much higher/lower than previous
            bool outlier = false;
            if (prev_y >= 0 && abs(y - prev_y) >= 90) {
              outlier = true;
            }

            if (!outlier) {
              // Calculate green/white percentage of pixels below
              float percent_green_or_white = 0.0;
              for (int c = 0; c < 200 && y + c < iparams_.height; c += 4) {
                Color col = ColorTableMethods::xy2color(img_, colorTable, x, y + c, iparams_.width);
                if (col == c_FIELD_GREEN || col == c_WHITE) {
                  percent_green_or_white += 1.0;
                }
              }
              percent_green_or_white /= 50;

              if (percent_green_or_white > 0.93) {
                field_edge_points.push_back(cv::Point(x, iparams_.height - y));
                prev_y = y;
              }
            }
          }
          fieldEdgeRunLengths[x/hstep_]++;	
        } 

        else {
          if (fieldEdgeRunLengths[x/hstep_] < 15)
            fieldEdgeRunLengths[x/iparams_.width] = 0;
        }
          
      }
      */

        
        if(c == c_ORANGE && !horizon_.isAbovePoint(x, y)) continue;
        if(c == c_WHITE && !horizon_.isAbovePoint(x,y)) c = c_ROBOT_WHITE; // We shouldn't be handling lines above the horizon
        
        if (fillGreenPct_) {
          if (c == c_FIELD_GREEN){
            if(camera_ == Camera::TOP) vblocks_.robot_vision->topGreenPct++;
            else vblocks_.robot_vision->bottomGreenPct++;
          }
          else if (c == c_UNDEFINED){
            if (camera_ == Camera::TOP) vblocks_.robot_vision->topUndefPct++;
            else vblocks_.robot_vision->bottomUndefPct++;
          }
        }
        segImg_[iparams_.width * y + x] = c;
      }
    }
  }
  if(fillGreenPct_) {
    if(camera_ == Camera::TOP){
      vblocks_.robot_vision->topGreenPct /= pixCount;
      vblocks_.robot_vision->topUndefPct /= pixCount;
    }
    else {
      vblocks_.robot_vision->bottomGreenPct /= pixCount;
      vblocks_.robot_vision->bottomUndefPct /= pixCount;
    }
  }

#ifdef TOOL

//  greenFile.close();
//  whiteFile.close();
//  grayFile.close();
//  blackFile.close();

#endif 


}

cv::Mat ColorSegmenter::extractRawMat(const ROI& roi) const {
  int xstep = std::max(roi.xstep, 2); // Step by at least 2 to maximize read efficiency
  roi.mat = cv::Mat(roi.height(), roi.width(), CV_8UC3);
  auto image = yuview::YUVImage::CreateFromRawBuffer(img_, iparams_.width, iparams_.height);
  for(int y = roi.ymin; y <= roi.ymax; y += roi.ystep) {
    //TODO: optimize
    for(int x = roi.xmin + roi.xmin % 2; x <= roi.xmax; x += xstep) {
      if(x >= iparams_.width) break;
      auto pixel = image.read(x, y);
      roi.mat.at<cv::Vec3b>(y - roi.ymin, x - roi.xmin) = cv::Vec3b(pixel.y0, pixel.u, pixel.v);
      if(x + 1 <= roi.xmax && x + 1 < iparams_.width)
        roi.mat.at<cv::Vec3b>(y - roi.ymin, x + 1 - roi.xmin) = cv::Vec3b(pixel.y1, pixel.u, pixel.v);;
    }
  }
  if(roi.ystep > 1 || roi.xstep > 1) // Don't use the local xstep since it's always at least 2
    cv::resize(roi.mat, roi.mat, cv::Size(), 1.0/xstep, 1.0/roi.ystep, cv::INTER_NEAREST);
  return roi.mat;
}

cv::Mat ColorSegmenter::extractHighresGrayscaleMat(const ROI& roi) const {
  int xstep = std::max(roi.xstep, 2); // Step by at least 2 to maximize read efficiency
  roi.mat = cv::Mat(roi.height(), roi.width(), CV_8UC1);
  auto image = yuview::YUVImage::CreateFromRawBuffer(img_, iparams_.width, iparams_.height);
  for(int y = roi.ymin; y <= roi.ymax; y += roi.ystep) {
    //TODO: optimize
    for(int x = roi.xmin + roi.xmin % 2; x <= roi.xmax; x += xstep) {
      if(x >= iparams_.width) break;
      auto pixel = image.read(x, y);
      roi.mat.at<uchar>(y - roi.ymin, x - roi.xmin) = pixel.y0;
      if(x + 1 <= roi.xmax && x + 1 < iparams_.width)
        roi.mat.at<uchar>(y - roi.ymin, x + 1 - roi.xmin) = pixel.y1;
    }
  }
  if(roi.ystep > 1 || roi.xstep > 1) // Don't use the local xstep since it's always at least 2
    cv::resize(roi.mat, roi.mat, cv::Size(), 1.0/xstep, 1.0/roi.ystep, cv::INTER_NEAREST);
  roi.extracted = true;
  return roi.mat;
}

cv::Mat ColorSegmenter::extractGrayscaleMat(const ROI& roi) const {
  int width = roi.xmax - roi.xmin;
  int height = roi.ymax - roi.ymin;
  cv::Mat mat = img_grayscale_(cv::Rect(roi.xmin, roi.ymin, width, height));
  cv::resize(mat, roi.mat, cv::Size(), 1.0/roi.xstep, 1.0/roi.ystep, cv::INTER_NEAREST);
  roi.extracted = true;
  return roi.mat;
}

cv::Mat ColorSegmenter::extractGrayscaleMat(const cv::Rect& rect, int xstep, int ystep) const {
  if (rect.x % xstep || rect.width % xstep) {
    std::cout << "ERROR in ColorSegmenter::extractGrayscaleMat: provided rect does is not aligned to provided xstep and ystep: " << rect << std::endl;
  }
  cv::Mat matLarge = img_grayscale_(rect);
  cv::Mat matSmall;
  cv::resize(matLarge, matSmall, cv::Size(), 1.0/xstep, 1.0/ystep, cv::INTER_NEAREST);
  return matSmall;
}

bool ColorSegmenter::startHighResScan(Color c, int hStepScale, int vStepScale) {
  std::vector<FocusArea> areas;
  if(!prepareFocusAreas(areas, c)) {
    tlog(28, "Threw out %i focus areas", areas.size());
    return false;
  }
  int totalArea = 0;
  int hstep = 1 << hStepScale, vstep = 1 << vStepScale;
  for(unsigned int i = 0; i < areas.size(); i++) {
    const FocusArea& area = areas[i];
    totalArea += area.area / hstep / vstep;
  }
  if((areas.size() > MAX_FOCUS_AREA_COUNT && totalArea > MAX_FOCUS_AREA) || totalArea > 10000) {
    tlog(28, "focus area: %i (max %i), count: %i (max %i)", totalArea, MAX_FOCUS_AREA, areas.size(), MAX_FOCUS_AREA_COUNT);
    return false;
  }
  // If there are too many focus areas it will slow things down a lot,
  // and this indicates that we're close to the scan object anyway.
  doingHighResScan_ = true;
  setStepScale(hStepScale, vStepScale);
  tlog(28, "Preparing high res scan on %i focus areas for %s", areas.size(), getName(c));
  classifyImage(areas, colorTable_);
  constructRuns(areas, 1 << c);
  return true;
}

bool ColorSegmenter::startHighResBallScan() {
  didHighResBallScan = true;
  return startHighResScan(c_ORANGE, 0, 0);
}

bool ColorSegmenter::startHighResGoalScan() {
  return startHighResScan(c_YELLOW, 1, 2);
}

void ColorSegmenter::completeHighResScan() {
  setStepScale(iparams_.defaultHorizontalStepScale, iparams_.defaultVerticalStepScale);
  doingHighResScan_ = false;
}

void ColorSegmenter::setHorizon(HorizonLine horizon) {
  horizon_ = horizon;
}

void ColorSegmenter::clearPoints(int colorFlags) {
  // Reset Vertical Point Counts
  for (int z = 0; z < Color::NUM_Colors; z++) {
    if(!isInFlags(z, colorFlags)) continue;
    memset(verticalPointCount[z], 0, sizeof(uint32_t) * iparams_.width);
  }

  // Reset Horizontal Point Counts
  for (int z = 0; z < Color::NUM_Colors; z++) {
    if(!isInFlags(z, colorFlags)) continue;
    memset(horizontalPointCount[z], 0, sizeof(uint32_t) * iparams_.height);
  }
}

void ColorSegmenter::constructRuns(int colorFlags) {
  clearPoints(colorFlags);
  FocusArea area(0, 0, iparams_.width - 1, iparams_.height - 1);
  constructRuns(area, colorFlags);
}

void ColorSegmenter::constructRuns(const std::vector<FocusArea>& areas, int colorFlags) {
  clearPoints(colorFlags);
  for(unsigned int i = 0; i < areas.size(); i++)
    constructRuns(areas[i], colorFlags);
}

void ColorSegmenter::constructRuns(const FocusArea& area, int colorFlags) {
  int x = 0, y = 0;

  // Reset stuff used to track vertical line points in horizontal scan
  uint8_t vRecent[iparams_.width];
  memset(vRecent, c_UNDEFINED, sizeof(uint8_t) * iparams_.width);
  uint8_t vRunClr[iparams_.width]; // Running line region color
  memset(vRunClr, c_UNDEFINED, sizeof(uint8_t) * iparams_.width);
  uint16_t vStartY[iparams_.width]; // Start of running line region
  memset(vStartY, -1, sizeof(uint16_t) * iparams_.width);


  uint8_t vLastRunClr[iparams_.width];  // Previous running line region color
  memset(vLastRunClr, c_UNDEFINED, sizeof(uint8_t) * iparams_.width);


  VisionPoint *prevVPointArray[iparams_.width];    // Pointer to previous run point
  for (int i = 0; i < iparams_.width; i++){
    prevVPointArray[i] = NULL;
  }

  for (y = area.y1; y <= area.y2; y += vstep_) {
    if(!SEGMENT_ABOVE_HORIZON) {
      int midX = (area.x1 + area.x2) / 2;
      if(!horizon_.isAbovePoint(midX, y)) continue;
    }

    hGreenPosition[y] = (uint16_t)-1;
    uint8_t hRecent = c_UNDEFINED;
    uint8_t hRunClr = c_UNDEFINED;
    uint16_t hStartX = 0;

    VisionPoint *prevHPoint = NULL;
    uint8_t hLastClr = c_UNDEFINED;

    bool ballRun = false;
    uint16_t ballHStartX = 0;

    for (x = area.x1; x <= area.x2; x += hstep_) {
      uint8_t c = segImg_[iparams_.width * y + x];

      // HORIZONTAL

      // Segment Finishes - when 2 consecutive pixels are different
      // Allow runs of length 1 for the ball




      if (hRunClr != c && (hRunClr != hRecent || hRunClr == c_ORANGE || c == c_ORANGE) && isInFlags(hRunClr, colorFlags)) {
        if (hRunClr == c_FIELD_GREEN) {
          hGreenPosition[y] = hStartX;
        } 
        if (hRunClr == c_BLUE || hRunClr == c_PINK || hRunClr == c_YELLOW || hRunClr == c_ORANGE || hRunClr == c_FIELD_GREEN || hRunClr == c_ROBOT_WHITE ||
                   (hRunClr == c_WHITE && hGreenPosition[y] != (uint16_t) -1)) {

          uint16_t valXI = hStartX;
          uint16_t valXF = (x - 2 * hstep_);

          // Allow runs of length 1 for the ball
          if(hRunClr == c_ORANGE)
            valXF = (x - hstep_);

          switch(hRunClr) {

          case c_WHITE: {
            bool aboveHorizon = horizon_.exists && (horizon_.gradient * valXI + horizon_.offset > y);
            bool widthTooHigh = valXF - valXI > 75 * iparams_.origFactor;
            bool onShoulder = bodyExclusionAvailable;
            uint16_t valAvgX = (valXI + valXF) / 2;
            uint16_t xIndex = ((NUM_BODY_EXCL_POINTS - 1) * valAvgX) / iparams_.width;
            onShoulder &=
              bodyExclusionSlope[xIndex] * valAvgX + bodyExclusionOffset[xIndex] < y;
            if (aboveHorizon || widthTooHigh || onShoulder) {
              break;
            }
          }

          default: {
            // Save line segment
//            cout << "VISION POINT " << static_cast<int>(hRunClr) << endl;       
            VisionPoint *lp =
              &horizontalPoint[hRunClr][y][horizontalPointCount[hRunClr][y]++];
            lp->xi = valXI;
            lp->xf = valXF;
            lp->dx = lp->xf - lp->xi + 1;
            lp->yi = lp->yf = y;
            lp->dy = 1;
            lp->isValid = true;
            if (hRunClr == c_WHITE && (hLastClr != c_FIELD_GREEN && prevHPoint != NULL)){
              lp->isValid = false;
//              cout << "NO GREEN BEFORE " << lp->xi << " " << lp->xf << " " << lp->yi << " " << lp->yf << " " << static_cast<int>(hLastClr) << endl;
//              cout << "     PREV " << prevHPoint->xi << " " << prevHPoint->xf << " " << prevHPoint->yi << " " << prevHPoint->yf << " " << endl;
            }
            lp->lbIndex = (uint16_t)-1; // Will get this information later


            if (hLastClr == c_WHITE){
              if (hRunClr != c_FIELD_GREEN){
                prevHPoint->isValid = false;
//              cout << "NO GREEN AFTER " << prevHPoint->xi << " " << prevHPoint->xf << " " << prevHPoint->yi << " " << prevHPoint->yf << " " << static_cast<int>(hRunClr) << endl;
              }
            }

            prevHPoint = lp;
            hLastClr = hRunClr;
          }

          }
        }

        hRunClr = c_UNDEFINED;

      }
      
      if (ballRun && !isBallPixel(c) && !isBallPixel(hRecent)){
        if (hGreenPosition[y] != (uint16_t) - 1){
          uint16_t valXI = ballHStartX;
          uint16_t valXF = (x - 2 * hstep_);
            VisionPoint *lp =
              &horizontalPoint[c_ORANGE][y][horizontalPointCount[c_ORANGE][y]++];
            lp->xi = valXI;
            lp->xf = valXF;
            lp->dx = lp->xf - lp->xi + 1;
            lp->yi = lp->yf = y;
            lp->dy = 1;
            lp->isValid = true;
            lp->lbIndex = (uint16_t)-1; // Will get this information later

        }
        ballRun = false;
      }


      // Segment Starts - when 2 consecutive pixels are same
      // Allow runs of length 1 for the ball
      if (c != hRunClr && (c == c_ORANGE || c == hRecent)) {
        hRunClr = c;
        if(x > hstep_)
          hStartX = x - hstep_;
        else
          hStartX = 0;
      }


      // Ball segment starts when we get 
      if (!ballRun && isBallPixel(c) && isBallPixel(hRecent)){
        ballRun = true;
        if (x > hstep_)
          ballHStartX = x - hstep_;
        else
          ballHStartX = 0;
      }


      hRecent = c;

      // VERTICAL

      uint8_t runClr = vRunClr[x];
      uint8_t recent = vRecent[x];

      VisionPoint *prevVPoint = prevVPointArray[x];
      uint8_t vLastClr = vLastRunClr[x];



      // Vision Point Segment finishes
      if (runClr != recent && runClr != c && isInFlags(runClr, colorFlags)) {
        if (runClr == c_FIELD_GREEN) {
          vGreenPosition[x] = vStartY[x];
        }
        if (runClr == c_BLUE || runClr == c_PINK || runClr == c_FIELD_GREEN || runClr == c_ROBOT_WHITE ||
                   (vGreenPosition[x] != (uint16_t)-1 && runClr == c_WHITE)) {

          uint16_t valX = x;
          uint16_t valYI = vStartY[x];
          uint16_t valYF = y - 4;

          // Introduce some special checks here
          switch(runClr) {

          case c_WHITE: {
            bool aboveHorizon = horizon_.exists && (horizon_.gradient * valX + horizon_.offset > valYI);
            bool widthTooHigh = valYF - valYI > 125 * iparams_.origFactor;
            bool onShoulder = bodyExclusionAvailable;
            uint16_t xIndex = ((NUM_BODY_EXCL_POINTS - 1) * valX) / iparams_.width;
            onShoulder &=
              bodyExclusionSlope[xIndex] * valX + bodyExclusionOffset[xIndex] < valYF;
            if (aboveHorizon || widthTooHigh || onShoulder) {
              break;
            }
          }

          default: {
            // Save Line Segment
            VisionPoint *lp =
              &verticalPoint[runClr][x][verticalPointCount[runClr][x]++];
            lp->xi = lp->xf = valX;
            lp->dx = 1;
            lp->yi = valYI;
            lp->yf = valYF;
            lp->dy = lp->yf - lp->yi + 1;
            lp->isValid = true;
            lp->lbIndex = (uint16_t)-1;
            
            
            if (runClr == c_WHITE && (vLastClr != c_FIELD_GREEN && prevVPoint != NULL)){
              lp->isValid = false; 
//              cout << "NO GREEN BEFORE " << lp->xi << " " << lp->xf << " " << lp->yi << " " << lp->yf << " " << static_cast<int>(vLastClr) << endl;
//              cout << "    PREV " << prevVPoint->xi << " " << prevVPoint->xf << " " << prevVPoint->yi << " " << prevVPoint->yf << " " << endl;
            }


            if (vLastClr == c_WHITE){
              if (runClr != c_FIELD_GREEN){
                prevVPoint->isValid = false;    
//              cout << "NO GREEN AFTER " << prevVPoint->xi << " " << prevVPoint->xf << " " << prevVPoint->yi << " " << prevVPoint->yf << " " << static_cast<int>(runClr) << endl;
              }
            }

            prevVPointArray[x] = lp;
            vLastRunClr[x] = runClr;


                   
          }

          }
        }
        vRunClr[x] = c_UNDEFINED;
      }

      // Vision Point Segment Starts
      if (c != runClr && c == recent) {
        vRunClr[x] = c;
        vStartY[x] = y - 2;
      }
      vRecent[x] = c;
    }

    // Last Horizontal Segment
    // Don't save the white ones
    if(!isInFlags(hRunClr, colorFlags)) continue;
    if (hRunClr == c_FIELD_GREEN) {
      hGreenPosition[y] = hStartX;
    }
    if (hRunClr == c_BLUE || hRunClr == c_YELLOW || hRunClr == c_ORANGE || hRunClr == c_FIELD_GREEN || hRunClr == c_ROBOT_WHITE) {
      // Save line segment
      completeHorizontalRun(hRunClr, y, hStartX, area.x2);
    }
  }

  // Last Vertical Segment
  // Don't save the white ones
  for (x = 0; x < iparams_.width; x += hstep_) {
    uint8_t runClr = vRunClr[x];
    if(!isInFlags(runClr, colorFlags)) continue;
    if (runClr == c_FIELD_GREEN) {
      vGreenPosition[x] = vStartY[x];
    }
    if (runClr == c_BLUE || runClr == c_PINK || runClr == c_FIELD_GREEN || runClr == c_ROBOT_WHITE) {
      // Save line segment
      completeVerticalRun(runClr, x, vStartY[x]);
    }
  }
}

void ColorSegmenter::preProcessPoints() {

// Don't really see how setting it to "point" removes it

  // White Points without green at the end are removed
  for (uint16_t x = 0; x < iparams_.width; x++) {
    for (uint16_t point = 0; point < verticalPointCount[c_WHITE][x]; point++) {
      if (verticalPoint[c_WHITE][x][point].yi > vGreenPosition[x]) {
        verticalPointCount[c_WHITE][x] = point;
        break;
      }
    }
  }

  // Horizontal white points without green at the end are also removed
  for (uint16_t y = 0; y < iparams_.height; y++) {
    for (uint16_t point = 0; point < horizontalPointCount[c_WHITE][y]; point++) {
      if (horizontalPoint[c_WHITE][y][point].xi > hGreenPosition[y]) {
        horizontalPointCount[c_WHITE][y] = point;
        break;
      }
    }
  }

  // This is done for the colored bands
  // Join 2 linepoints together which are in the same vertical line and very close.
/*

  // Sanmit: Robots don't have bands anymore... 

  if (vparams_.ALLOW_BAND_MERGE_POINTS) {
    unsigned char bandColors[] = {
      c_BLUE,
      c_PINK
    };

    for (uint16_t c = 0; c < 2; c++) {
      unsigned char color = bandColors[c];
      for (uint16_t x = 0; x < iparams_.width; x++) {
        uint16_t skip = 1;
        for (int16_t i = 0; i < ((int16_t)verticalPointCount[color][x]) - 1; i += skip) {
          skip = 1;
          for (uint16_t j = i + 1; j < verticalPointCount[color][x]; j++) {
            if (verticalPoint[color][x][j].yi - verticalPoint[color][x][i].yf <= vparams_.BAND_MERGE_POINT_DIST) {
              verticalPoint[color][x][i].yf = verticalPoint[color][x][j].yf;
              verticalPoint[color][x][i].dy = verticalPoint[color][x][i].yf - verticalPoint[color][x][i].yi + 1;
              verticalPoint[color][x][j].isValid = false;
              skip++;
            }
          }
        }
      }
    }
  }
*/  

}

void ColorSegmenter::preProcessGoalPoints() {

  // This is done for the goals
  // Join 2 linepoints together which are in the same vertical line and very close.
  if (vparams_.ALLOW_GOAL_MERGE_POINTS) {
    unsigned char goalColors[] = {
      c_YELLOW
    };

    for (uint16_t c = 0; c < 1; c++) {
      unsigned char color = goalColors[c];
      for (uint16_t x = 0; x < iparams_.height; x++) {
        uint16_t skip = 1;
        for (int16_t i = 0; i < ((int16_t)horizontalPointCount[color][x]) - 1; i += skip) {
          skip = 1;
          for (uint16_t j = i + 1; j < horizontalPointCount[color][x]; j++) {
            if (horizontalPoint[color][x][j].xi - horizontalPoint[color][x][i].xf <= vparams_.GOAL_MERGE_POINT_DIST) {
              horizontalPoint[color][x][i].xf = horizontalPoint[color][x][j].xf;
              horizontalPoint[color][x][i].dx = horizontalPoint[color][x][i].xf - horizontalPoint[color][x][i].xi + 1;
              horizontalPoint[color][x][j].isValid = false;
              skip++;
            }
          }
        }
      }
    }
  }

}

void ColorSegmenter::completeHorizontalRun(uint8_t &hRunClr, int y, int hStartX, int finX) {
  VisionPoint *lp = &horizontalPoint[hRunClr][y][horizontalPointCount[hRunClr][y]++];
  lp->xi = hStartX;
  lp->xf = ((finX >> hscale_) - 1) << hscale_;
  lp->dx = lp->xf - lp->xi + 1;
  lp->yi = lp->yf = y;
  lp->dy = 1;
  lp->isValid = true;
  lp->lbIndex = (uint16_t)-1;
  hRunClr = c_UNDEFINED;
}

void ColorSegmenter::completeVerticalRun(uint8_t &runClr, int x, int vStartY) {
  VisionPoint *lp =
    &verticalPoint[runClr][x][verticalPointCount[runClr][x]++];
  lp->xi = lp->xf = x;
  lp->dx = 1;
  lp->yi = vStartY;
  lp->yf = iparams_.height - vstep_;
  lp->dy = lp->yf - lp->yi + 1;
  lp->isValid = true;
  lp->lbIndex = (uint16_t)-1;
  runClr = c_UNDEFINED;
}

void ColorSegmenter::setStepScale(int h, int v){
    hstep_ = (1 << h);
    vstep_ = (1 << v);
    hscale_ = h;
    vscale_ = v;
}

void ColorSegmenter::getStepSize(int& h, int& v) const {
    h = hstep_;
    v = vstep_;
}

void ColorSegmenter::getStepScale(int& h, int& v) const {
    h = hscale_;
    v = vscale_;
}

bool ColorSegmenter::prepareFocusAreas(std::vector<FocusArea>& areas, Color c) {
  int hrange = FOCUS_RANGE_HORIZONTAL(c), vrange = FOCUS_RANGE_VERTICAL(c);
  for(int y = 0; y < iparams_.height; y += vstep_) {
    int ymin = std::max(0, y - vrange), ymax = std::min(iparams_.height - 1, y + vrange);
    for(unsigned int i = 0; i < horizontalPointCount[c][y]; i++) {
      VisionPoint* point = &horizontalPoint[c][y][i];
      int xmin = std::max(0, point->xi - hrange), xmax = std::min(iparams_.width - 1, point->xf + hrange);
      FocusArea area(xmin, ymin, xmax, ymax);
      areas.push_back(area);
    }
  }
  tlog(28, "%i initial focus areas found", areas.size());
  std::vector<FocusArea> final;
  switch(c) {
    case c_ORANGE: areas = FocusArea::merge(areas); break;
    case c_YELLOW: 
       areas = FocusArea::mergeVertical(areas, iparams_);
       for(unsigned int i = 0; i < areas.size(); i++)
         if(areas[i].height >= 100)
           final.push_back(areas[i]);
       areas = final;
       break;
    default: break;
  }
  tlog(28, "%i final focus areas found", areas.size());
  return areas.size() > 0;
}

float ColorSegmenter::colorPercentageInBox(int xmin, int xmax, int ymin, int ymax, Color c){
  
  int colorCount = 0;
  int totalCount = 0;
  // Convert to coordinates where we have classified pixels
  xmin = std::max(xmin, 0) - (xmin % hstep_);
  xmax = std::min(xmax, iparams_.width - 1) - (xmax % hstep_);
  ymin = std::max(ymin, 0) - (ymin % vstep_);
  ymax = std::min(ymax, iparams_.height - 1) - (ymax % vstep_);
  for (int x = xmin; x <= xmax; x+= hstep_){
    for (int y = ymin; y <= ymax; y+= vstep_){
      totalCount++;
      Color color = xy2color(x,y);
      if (color == c){
        colorCount++; 
      }
    }
  }

  float colorPct = (float)colorCount / (float)totalCount;
  if (!totalCount && ymin >= iparams_.height)
    colorPct = 1;     // This heuristic was inherited from old code. 
  
  return colorPct;
}


void ColorSegmenter::clearPreviousHighResScans() {
#ifdef TOOL
  bool imageLoaded = vblocks_.image->isLoaded();
  if (imageLoaded) // if a raw image is available
#endif
  {
    if (segImg_) {
      memset(segImg_, c_UNDEFINED, iparams_.size);
      tlog(20, "clearing seg image");
    }
    img_grayscale_ = Mat::zeros(iparams_.height, iparams_.width, CV_8UC1);
  }
}
