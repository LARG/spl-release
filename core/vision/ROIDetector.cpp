#include "ROIDetector.h"
#include <stdio.h>

#define getball() vblocks_.world_object->objects_[WO_BALL]

#define DEBUG_TIMING false
#define WRITE_INFO_TO_DISK false
#define HIGHRES_THRESHOLD_SMALL 75  // 45 should definitely be high res 
#define HIGHRES_THRESHOLD_MEDIUM 95 // 75? 65 should definitely be medium res

using namespace std;
using namespace cv;

ROIDetector::ROIDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter, FieldEdgeDetector& field_edge_detector) :
  DETECTOR_INITIALIZE, color_segmenter_(segmenter), field_edge_detector_(field_edge_detector) {

   xstep_ = 1 << iparams_.defaultHorizontalStepScale;
   ystep_ = 1 << iparams_.defaultVerticalStepScale;
}

bool ROIDetector::setImagePointers() {
  if(vblocks_.image == NULL) {
    printf("No image block loaded!\n");
    return false;
  }
  if(camera_ == Camera::TOP) {
    img_ = vblocks_.image->getImgTop();
    if(!img_) return false;
  }
  else {
    img_ = vblocks_.image->getImgBottom();
    if(!img_) return false;
  }
  return true;
}

int ROIDetector::chooseThreshold(int window, int bandIndex) {

  
  // 85 is a conservative number that works well indoors without creating too many extra roi points or selections
  // 65 is an aggressive threshold that works well, but produces a lot of ROI points and candidates. Finds more ROIs especially in natural light. 
  // Sometimes the bottom band (2) on bottom camera can be darker, especially when two robots are together and ball is at the feet. But otherwise it doesn't seem like the threshold should vary by distance... 

  return 65; 


 /* 
  if (camera_ == Camera::BOTTOM) {
    if (bandIndex == 2) {  // bottom of image
      return 100;
    }
    return 110;
  }

  if (window <= 3) {  // closer to top of image
    return 75;
  }
  if (window <= 5) {  // closer to middle of image
    return 80;
  }
  if (window <= 7) {  // closer to bottom of image
    return 90;
  }

  return 100;
*/  
}

float ROIDetector::chooseWindowFactor(int bandIndex) {

  // TODO: Sanmit
  // Smaller window sizes seem to be better overall. E.g. using window 7 on bottom seemed to be better... unless there were shadows. Outdoors it did slightly worse. 
  // Need to see on a robot whether smaller is better for framerate. 

  if (camera_ == Camera::TOP) {
    return 1.6;
  }

  switch (bandIndex) {
    case 0:
      return 1.4;  // top of image
    case 1:
      return 2.0;  // middle of image
  }
  return 2.2;  // bottom of image
}

void ROIDetector::findBallROIs() {
  VisionTimer::Start(60, "ROIDetector(%s)::findBallROIs", camera_);

  points.clear();
  seams.clear();
  ballROIs.clear();


  VisionTimer::Start(60, "ROIDetector(%s)::hull_generation", camera_);

  vector<FieldEdgePoint> hullPoints = field_edge_detector_.hullPointCands;
  
  int hullCounter = 0;
  int leftHullIndex = 0;
  int rightHullIndex = 1272;
  int hullThreshold = 2;

  if (camera_==Camera::TOP){
//   printf("Num hull points: %d\n", hullPoints.size());
     for (int p = 0; p < hullPoints.size(); p++){
        
       if (hullPoints[p].valid && !hullPoints[p].below){
        hullCounter++;
        if (hullCounter >= hullThreshold){
          rightHullIndex = hullPoints[p].x;
          break;
        }
       }
//        printf("Hull point: %d %d\n", hullPoints[p].x, hullPoints[p].hullY);
//        cout << "Hull point: " << hullPoints[p].x << " " << hullPoints[p].hullY << endl;
     }
     hullCounter = 0;
     for (int p = hullPoints.size()-1; p>= 0; p--){
       if (hullPoints[p].valid && !hullPoints[p].below){
        hullCounter++;
        if (hullCounter >= hullThreshold){
          leftHullIndex = hullPoints[p].x;
          break;
        }
       }
     }
   }

   leftHullIndex = std::max(160, leftHullIndex);
   rightHullIndex = std::min(1120, rightHullIndex);

//  printf("Left Hull %d Right Hull %d\n", leftHullIndex, rightHullIndex);




  vector<FieldEdgePoint> cvxHullPoints = field_edge_detector_.hullPoints; 

  // Need to evaluate left and right sides of the image outside of (leftHullIndex, rightHullIndex)
  bool rightEdgeOK = true;
  bool leftEdgeOK = true;
  if (camera_==Camera::TOP){
    

    // First check if there are 5 valid hullpointCands on either side. If there are, then that side is done, and we can keep the detected field edge boundary
    for (int p = 0; p < 5; p++){
      if (!hullPoints[p].valid || hullPoints[p].below){
        rightEdgeOK = false;
      }
    }
    // hullPoints should be greater than size 5
    for (int p = hullPoints.size()-1; p >= hullPoints.size() - 5; p--){
      if (!hullPoints[p].valid || hullPoints[p].below){
        leftEdgeOK = false;
      }
    }
  
    // If the edge was OK, then it's ok to use the hullY's calculated already for them. 
    // Otherwise, we need to recompute hullYs by projecting new field edge lines
   
    // Need at least 5 cvxHullPoints???

   
    if (!leftEdgeOK){

      int cvxHPCount = 0;
      int cvxHPIndex = 0; 

      // Check the number of cvxHullPoints within the leftHullIndex
      for (int i = 0; i < cvxHullPoints.size(); i++){
        if (cvxHullPoints[i].x <= leftHullIndex){
          cvxHPCount++;
        }
        else {
          cvxHPIndex = i;
          break;
        }
        if (cvxHPCount >= 3){
          cvxHPIndex = i;
          break;
        }
      }

      // If there are 3, use the slope between points 3 & 4 to project points for points before 3
      if (cvxHPCount >= 3){

        // Another point exists
        if (cvxHPIndex+1 < cvxHullPoints.size()){
          LineSegment line(cvxHullPoints[cvxHPIndex], cvxHullPoints[cvxHPIndex+1]);
          for (int p = hullPoints.size()-1; p >= 0; p--){
            if (hullPoints[p].x < cvxHullPoints[cvxHPIndex].x){
              hullPoints[p].hullY = line.getYGivenX(hullPoints[p].x); 
            }
            else {
              break;
            }
          }
          leftEdgeOK = true;
        }
      }
      // Otherwise, find the first cvxHullPoint outside of the region and project a new line between the first and second cvxHullPoints outside the region. Let's be conservative for now and apply it to just the original region points (though maybe we will want to extend it?)
      else {
         
        if (cvxHPIndex+1 < cvxHullPoints.size()){
          LineSegment line(cvxHullPoints[cvxHPIndex], cvxHullPoints[cvxHPIndex+1]);
          for (int p = hullPoints.size()-1; p >= 0; p--){
            if (hullPoints[p].x < leftHullIndex){
              hullPoints[p].hullY = line.getYGivenX(hullPoints[p].x);
            }
            else {
              break;
            }
          }
          leftEdgeOK = true;
        }
      }
    }

  
    if (!rightEdgeOK){

      int cvxHPCount = 0;
      int cvxHPIndex = cvxHullPoints.size()-1; 

      // Check the number of cvxHullPoints within the rightHullIndex
      for (int i = cvxHullPoints.size()-1; i >= 0; i--){
        if (cvxHullPoints[i].x >= rightHullIndex){
          cvxHPCount++;
        }
        else {
          cvxHPIndex = i;
          break;
        }
        if (cvxHPCount >= 3){
          cvxHPIndex = i;
          break;
        }
      }

      // If there are 3, use the slope between points 3 & 4 to project points for points before 3
      if (cvxHPCount >= 3){

        // Another point exists
        if (cvxHPIndex-1 >= 0){
          LineSegment line(cvxHullPoints[cvxHPIndex-1], cvxHullPoints[cvxHPIndex]);
          for (int p = 0; p < hullPoints.size(); p++){
            if (hullPoints[p].x > cvxHullPoints[cvxHPIndex].x){
              hullPoints[p].hullY = line.getYGivenX(hullPoints[p].x); 
            }
            else {
              break;
            }
          }
          rightEdgeOK = true;
        }
      }
      // Otherwise, find the first cvxHullPoint outside of the region and project a new line between the first and second cvxHullPoints outside the region. Let's be conservative for now and apply it to just the original region points (though maybe we will want to extend it?)
      else {
         
        if (cvxHPIndex-1 >= 0){
          LineSegment line(cvxHullPoints[cvxHPIndex-1], cvxHullPoints[cvxHPIndex]);
          for (int p = 0; p < hullPoints.size(); p++){
            if (hullPoints[p].x > rightHullIndex){
              hullPoints[p].hullY = line.getYGivenX(hullPoints[p].x);
            }
            else {
              break;
            }
          }
          rightEdgeOK = true;
        }
      }
    }
  }


  VisionTimer::Stop("ROIDetector(%s)::hull_generation", camera_);



  // if ball was already detected by bottom camera, and we're in top cam, done
  WorldObject &ball = getball();
  if(ball.seen && !ball.fromTopCamera && camera_ == Camera::TOP) {
    tlog(60, "Ball was seen already in the bottom camera, bailing out.");
    VisionTimer::Stop("ROIDetector(%s)::findBallROIs", camera_);
    return;
  }

  // make sure we have an image
  if (!setImagePointers())
    return;

  // create a grayscale cv::Mat of the whole image, subsampled by xstep and ystep
  VisionTimer::Start(60, "ROIDetector(%s)::resizeMat", camera_);
  cv::Mat mat;
  cv::resize(color_segmenter_.img_grayscale(), mat, cv::Size(), 1.0 / xstep_, 1.0 / ystep_, cv::INTER_NEAREST); 
  VisionTimer::Stop("ROIDetector(%s)::resizeMat", camera_);

  // write info to disk
  if (WRITE_INFO_TO_DISK) {
    char* logname = "vision_ball_bottom";
    std::string filepath = util::ssprintf("%s/BlobImages/%s/top/image_%03i.png", util::env("NAO_HOME"), logname, frame_counter_);
    if (camera_ == Camera::BOTTOM) {
      filepath = util::ssprintf("%s/BlobImages/%s/bottom/image_%03i.png", util::env("NAO_HOME"), logname, frame_counter_);
    }
    std::cout << "writing info to disk: " << filepath << std::endl;
    cv::imwrite(filepath, mat);
    cmatrix_.writeCameraMatrix(logname, frame_counter_, camera_ == Camera::BOTTOM);
    horizon_.writeHorizon(logname, mat.cols*xstep_, frame_counter_, camera_ == Camera::BOTTOM);    
  }

#ifdef TOOL
  binaryImg = cv::Mat(mat.rows, mat.cols, CV_8UC1);
  binaryImg.setTo(cv::Scalar(255));
#endif

  VisionTimer::Start(60, "ROIDetector(%s)::adaptiveThreshold", camera_);
  int numBands = 3;
  int mid_x = mat.cols/2;
  float worldHeight = 80.0;  // millimeters
  float worldWidth = 30.0;   // millimeters
  int horizon_offset = 0; // Causes segfaults when horizon below image frame -> (int)(horizon_.getLargestYCoord(mat.cols*xstep_)/xstep_);
  int height = (mat.rows - horizon_offset)/numBands;
  int divisionPadding = std::ceil(0.2*height); // px
  for(int bandIndex = 0; bandIndex < numBands; ++bandIndex) {
    cv::Rect rect(
      0, // x
      std::max(0, horizon_offset + bandIndex*height - divisionPadding),  // y
      mat.cols,  // width
      height + divisionPadding // height
    );

    seams.push_back(rect.y);

/*
    int mid_y = rect.y + rect.height/2;

    float blob_size = cmatrix_.getExpectedCameraWidth(mid_x*xstep_, mid_y*ystep_, worldHeight, worldWidth);
    if (camera_ == Camera::BOTTOM) {
      blob_size = 0.1696*blob_size + 14.097;
    } else if (camera_ == Camera::TOP) {
      blob_size = 12.02 + 1.025*blob_size;
    }
    blob_size /= xstep_;


    // SN: This window code produces too many points. And the clustering algorithm is waaay too slow. 
    float window_factor = chooseWindowFactor(bandIndex);
    int window = blob_size*window_factor;

    // window must be at least 3 and must be odd
    window = std::max(window, 3);
    if(window % 2 == 0) {
      window++;
    }
*/

    // Previous window code produces too many points. This seems to work about as well with much fewer points. 
    // You need a larger window for outdoors / natural lighting because of the shadows. 
    int window;
    if (camera_==Camera::BOTTOM){
      window = 5;
    }
    else{
      
      // Using different since something weird going on with ROI point verification below.. but we really want 5 for the top cam...  
      if (bandIndex == 0){
        window = 5;
      }
      else {
        window = 3;
      }
    }

    int threshold = chooseThreshold(window, bandIndex);

//    tlog(60, "%s band index %d: window_factor = %f || window = %d ||threshold = %d",
//      Camera::c_str(camera_), bandIndex, window_factor, window, threshold);

    // adaptive threshold image with high threshold
    cv::Mat binaryImgPortion;
    cv::adaptiveThreshold(mat(rect), binaryImgPortion, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, window, threshold);

#ifdef TOOL
    binaryImgPortion.copyTo(binaryImg(rect));
#endif

  // TODO: SN - Use Elad's field edge boundary stuff to filter ROI points (i.e. remove points that are off the field, so we will have to cluster fewer points)
  // This is important because the clustering is very slow (O(n^2)) and quickly becomes too slow after ~70 points (worse case ~3.5ms)

  // TODO: something is not right here. Sometimes pixels that have green below are still allowed to be ROI points... Nevermind it's the padding... adding a hack to ignore padding on top cam top band  
  // Nevermind, something is still off... 
    // find dark pixels
    int priorPointSize = points.size();
    for(int row = 0; row < binaryImgPortion.rows; ++row) {
      for(int col = 0; col < binaryImgPortion.cols; ++ col) {
//        printf("Index into hulls: %d\n", (binaryImgPortion.cols - col -1)); // *xstep_
        if(binaryImgPortion.at<unsigned char>(row, col) < 255
          && getSegPixelValueAt(col*xstep_, (rect.y + row)*ystep_) != c_FIELD_GREEN
          && ( (camera_ == Camera::TOP && bandIndex == 0 && (rect.y + row) < height - divisionPadding) || 
          (getSegPixelValueAt(std::max(col-1, 0)*xstep_, (rect.y + row)*ystep_) != c_FIELD_GREEN
          && getSegPixelValueAt(col*xstep_, (std::max(rect.y + row-1, 0))*ystep_) != c_FIELD_GREEN
          && getSegPixelValueAt(std::min(col+1, mat.cols-1)*xstep_, (rect.y + row)*ystep_) != c_FIELD_GREEN
          && getSegPixelValueAt(col*xstep_, (std::min(rect.y + row+1, mat.rows-1))*ystep_) != c_FIELD_GREEN ))
          ) {
        
        // Crude hull point filtering
        // We always take points on the left and right 12.5% of the image or the second deepest valid hull point from the side, whichever is larger. 

        if ((camera_==Camera::TOP) && (hullPoints[(binaryImgPortion.cols - col - 1)].hullY > (rect.y+row)*ystep_)){

          // Skip this point
          if (col*xstep_ < leftHullIndex && leftEdgeOK){
            continue;
          }
          if (col*xstep_ > rightHullIndex && rightEdgeOK){
            continue;
          }
          if (col*xstep_ >= leftHullIndex && col*xstep_ <= rightHullIndex){
            continue;
          }
        }
          
//        if ((camera_==Camera::TOP) && (col*xstep_ >= std::max(160, leftHullIndex) && col*xstep_ <= std::min(1120, rightHullIndex)) && hullPoints[(binaryImgPortion.cols - col - 1)].hullY > (rect.y+row)*ystep_){
          // Skip this point
//          continue;
//        }

          points.push_back(cv::Point(col, rect.y + row));
        }
      }
    }
    tlog(60, "found %i points for threshold window", points.size());
//    printf("ROI points in band %d : %d\n", bandIndex, points.size()-priorPointSize);
  }
//
//  printf("Total ROI points in frame: %d\n", points.size());
//#ifdef TOOL
  nROIPoints += points.size();
//#endif    
  
  VisionTimer::Stop("ROIDetector(%s)::adaptiveThreshold", camera_);

//  printf("Average ROI points per frame (%s): %f\n", (camera_ == Camera::TOP) ? "top" : "bottom", nROIPoints * 1.0 / frame_counter_);

  // cluster pixels
  VisionTimer::Start(60, "ROIDetector(%s)::cluster", camera_);
  std::vector<int> labels;
  int n_labels = cv::partition(points, labels,
    [&](const cv::Point& lhs, const cv::Point& rhs) {
      float worldHeight = 0.0;
      float worldWidth = 100.0;
      int x = (lhs.x + rhs.x)/2;
      int y = (lhs.y + rhs.y)/2;

      float ball_size = cmatrix_.getExpectedCameraWidth(x*xstep_, y*ystep_, worldHeight, worldWidth);
      if (camera_ == Camera::BOTTOM) {
        ball_size = -0.0003*std::pow(ball_size, 2) + 0.1762*ball_size + 54.069;
      } else {
        ball_size = 12.02 + 1.025*ball_size;
      }
      ball_size /= (xstep_ + ystep_)/2.0;

      int maxDist = std::ceil(ball_size*0.65);
      return std::pow(lhs.x - rhs.x, 2) + std::pow(lhs.y - rhs.y, 2) < std::pow(maxDist, 2);
  }); 
  VisionTimer::Stop("ROIDetector(%s)::cluster", camera_);

  // compute the centroid of the clusters
  std::vector<int> xCentroids(n_labels, 0);
  std::vector<int> yCentroids(n_labels, 0);
  std::vector<int> totals(n_labels, 0);
  for (int i = 0; i < labels.size(); ++i) {
    int label = labels[i];
    xCentroids[label] += points[i].x;
    yCentroids[label] += points[i].y;
    ++totals[label];
  }

  // make ROI objects from clustered pixels
  // put their center at the cluster centroid
  // width and height will be wrong, but padding is added later to correct it
  VisionTimer::Start(60, "ROIDetector(%s)::makingROIs", camera_);
  std::vector<ROI> unfilteredROIs(n_labels, ROI(10000, -1, 100000, 1, camera_));
  for(int i = 0; i < n_labels; ++i) {
    auto& roi = unfilteredROIs[i];
    roi.xmin = xCentroids[i]/totals[i] - 1;
    roi.xmax = xCentroids[i]/totals[i] + 1;
    roi.ymin = yCentroids[i]/totals[i] - 1;
    roi.ymax = yCentroids[i]/totals[i] + 1;
    roi.xstep = xstep_;
    roi.ystep = ystep_;
    roi.hscale = 3;
    roi.vscale = 2;
    if(camera_ == Camera::BOTTOM) {
      roi.hscale = 1;
      roi.vscale = 1;
    }
#ifdef TOOL
    roi.segmenter = &color_segmenter_;
#endif
  }

  // scale the cooridinates to original resolution
  for(auto& roi : unfilteredROIs) {
    roi.xmin *= xstep_;
    roi.xmax *= xstep_;
    roi.ymin *= ystep_;
    roi.ymax *= ystep_;
  }

  // add padding to ROIs
  if(camera_ == Camera::TOP) {
    padRoisTopCamera(unfilteredROIs);
  }
  else {
    padRoisBottomCamera(unfilteredROIs);
  }

  // filter out bad ROIs (above horizon)
  tlog(60, "filtering %i ball rois", unfilteredROIs.size());
  const float greenBelowThresh = 0.5;
  const float greenInsideThresh = 0.85;
  for(const auto& roi : unfilteredROIs) {

    tlog(60, "checking roi: %s", roi);
    if(!horizon_.isAbovePoint(roi.xmin, roi.ymin) || !horizon_.isAbovePoint(roi.xmax, roi.ymin)) {
      // std::cout << "throwing out (" << (roi.xmax-roi.xmin) << ", " << (roi.ymax-roi.ymin) << ") because horizon" << std::endl;
      tlog(60, "throwing out roi due to horizon: %s", roi);
      continue;
    }

    // filter out ROI's that don't have green below only for top camera
    // TODO: we can probably make this box below larger, or be more aggressive with the threshold
    // This might not be necessary anymore with field edge detection
    if (camera_ == Camera::TOP){
      float greenBelowPct = checkGreenBelowPct(roi); 
      if (greenBelowPct < greenBelowThresh){
        tlog(60, "throwing out roi %s due to green below %f < %f\n", roi, greenBelowPct, greenBelowThresh);
        continue;
      }
    }

    // filter out ROI's that have too much green inside.  
    float greenInsidePct = checkGreenInsidePct(roi);
    if (greenInsidePct > greenInsideThresh){
        tlog(60, "throwing out roi %s due to green inside %f > %f\n", roi, greenInsidePct, greenInsideThresh);
        continue;
    }


    // filter out ROIs that are more than 7 meters away (very unlikely we will see anyways, and these tend to give a lot of noise)
    float roiDist = cmatrix_.groundDistance(cmatrix_.getWorldPosition(roi.centerX(), roi.ymax));
    if (roiDist > 7000){
      tlog(60, "throwing out roi %s due to distance %f > 7000", roi, roiDist);
      continue;
    }

    ballROIs.push_back(roi);
  }
  tlog(60, "kept %i ball rois", ballROIs.size());

//  printf("ROI's found in this frame (%s): %d\n", (camera_==Camera::TOP) ? "top" : "bottom", ballROIs.size());

#ifdef TOOL
  nROIRegions += ballROIs.size();
//  printf("Average number ROIs (%s): %f\n", (camera_==Camera::TOP) ? "top" : "bottom", nROIRegions * 1.0 / frame_counter_);
#endif

  for(auto& roi : ballROIs) {
    // high res scan
    if(camera_ == Camera::TOP && roi.width() < HIGHRES_THRESHOLD_SMALL) {
      roi.setScale(1, 1);
    }
    else if(camera_ == Camera::TOP && roi.width() < HIGHRES_THRESHOLD_MEDIUM) {
      roi.setScale(2, 2);
    }

    // align roi so that classifying image and subsequently resizing image will hit the right pixels
    roi.xmin -= roi.xmin % xstep_;
    roi.xmax += (xstep_ - (roi.xmax % xstep_));
    roi.ymin -= roi.ymin % ystep_;
    roi.ymax += (ystep_ - (roi.ymax % ystep_));

    // add small padding
    roi.xmin -= roi.xstep;
    roi.xmax += roi.xstep;
    roi.ymin -= roi.ystep;
    roi.ymax += roi.ystep;

    // clip the bounding box to the image extent
    roi.xmin = std::max(roi.xmin, 0);
    roi.ymin = std::max(roi.ymin, 0);
    roi.xmax = std::min(roi.xmax, mat.cols*xstep_);
    roi.ymax = std::min(roi.ymax, mat.rows*ystep_);
  }

#ifdef TOOL
  int pix_count = 0;
  for(auto& roi : ballROIs) {
    pix_count += (roi.xmax - roi.xmin)*(roi.ymax - roi.ymin)/roi.xstep/roi.ystep;
  }
  pixel_counter_ += pix_count;
  max_pixel_count_ = std::max(pix_count, max_pixel_count_);

  tlog(60, "number of points = %s", points.size());
  tlog(60, "number of pixels = %s", pix_count);
  tlog(60, "avg number of pixels = %s", (pixel_counter_/frame_counter_));
  tlog(60, "max number of pixels = %s", max_pixel_count_);
#endif

  frame_counter_++;
  VisionTimer::Stop("ROIDetector(%s)::makingROIs", camera_);

  VisionTimer::Stop("ROIDetector(%s)::findBallROIs", camera_);

}

float ROIDetector::checkGreenBelowPct(const ROI &roi){

  // We use the default step sizes because the area under the ball shouldn't be high-res scanned
  int hstep = 1 << iparams_.defaultHorizontalStepScale, vstep = 1 << iparams_.defaultVerticalStepScale;

  int ymin = roi.ymax;
  int ymax = roi.ymax + (roi.height() / 2);
  int xmin = roi.xmin;
  int xmax = roi.xmax;
  
  // Make sure indices start on pixels that are classified and within image boundaries
  ymin -= ymin % vstep;
  ymax += vstep - ymax % vstep;
  xmin -= xmin % hstep;
  xmax += hstep - xmax % hstep;

  xmin = std::max(xmin, 0);
  xmax = std::min(xmax, iparams_.width - 1);
  ymin = std::max(ymin, 0);
  ymax = std::min(ymax, iparams_.height - 1);

  int green = 0, total = 0;
  for (int x = xmin; x <= xmax; x += hstep){
    for (int y = ymin; y <= ymax; y += vstep){
      int c = getSegPixelValueAt(x, y);
      total++;
      green += (c == c_FIELD_GREEN);
    }
  }

  if(!total && ymin >= iparams_.height){
    return 1;
  }

  return ((float)(green)/total);

}


// Use this to eliminate ROI's that are mostly just field edges
float ROIDetector::checkGreenInsidePct(const ROI &roi){

  //
  int hstep = 1 << iparams_.defaultHorizontalStepScale, vstep = 1 << iparams_.defaultVerticalStepScale;

  int ymin = roi.ymin;
  int ymax = roi.ymax;
  int xmin = roi.xmin;
  int xmax = roi.xmax;
  
  // Make sure indices start on pixels that are classified and within image boundaries
  ymin -= ymin % vstep;
  ymax += vstep - ymax % vstep;
  xmin -= xmin % hstep;
  xmax += hstep - xmax % hstep;

  xmin = std::max(xmin, 0);
  xmax = std::min(xmax, iparams_.width - 1);
  ymin = std::max(ymin, 0);
  ymax = std::min(ymax, iparams_.height - 1);

  int green = 0, total = 0;
  for (int x = xmin; x <= xmax; x += hstep){
    for (int y = ymin; y <= ymax; y += vstep){
      int c = getSegPixelValueAt(x, y);
      total++;
      green += (c == c_FIELD_GREEN);
    }
  }

  if(!total){
    return 0;   // Should never happen
  }

  return ((float)(green)/total);

}



// make roi bounding boxes larger
void ROIDetector::padRoisTopCamera(std::vector<ROI>& rois) {
  int worldHeight = 0.0;
  int worldWidth = 100;

  for(auto& roi : rois) {
    int x = (roi.xmin + roi.xmax)/2;
    int y = (roi.ymin + roi.ymax)/2;
    int expectedWidth = cmatrix_.getExpectedCameraWidth(x, y, worldHeight, worldWidth);
    // std::cout << roi << "\texpectedWidth = " << expectedWidth;
    expectedWidth = 12.02 + 1.025*expectedWidth;
    // std::cout << "\tafter = " << expectedWidth << std::endl;

    roi.xmin = std::min(roi.xmin, x - expectedWidth/2);
    roi.xmax = std::max(roi.xmax, x + expectedWidth/2);
    roi.ymin = std::min(roi.ymin, y - expectedWidth/2);
    roi.ymax = std::max(roi.ymax, y + expectedWidth/2);

    int height = roi.ymax - roi.ymin;
    int width = roi.xmax - roi.xmin;
    roi.ymax += 0.25*height;
    roi.xmin -= 0.15*width;
    roi.xmax += 0.15*width;
  }
}

void ROIDetector::padRoisBottomCamera(std::vector<ROI>& rois) {
  int worldHeight = 0.0;
  int worldWidth = 100;

  for(auto& roi : rois) {
    int x = (roi.xmin + roi.xmax)/2;
    int y = (roi.ymin + roi.ymax)/2;
    int expectedWidth = cmatrix_.getExpectedCameraWidth(x, y, worldHeight, worldWidth);
    expectedWidth = -0.0003*std::pow(expectedWidth, 2) + 0.1762*expectedWidth + 54.069;

    roi.xmin = std::min(roi.xmin, x - (int)(1.3*expectedWidth/2));
    roi.xmax = std::max(roi.xmax, x + (int)(1.3*expectedWidth/2));
    roi.ymin = std::min(roi.ymin, y - (int)(1.3*expectedWidth/2));
    roi.ymax = std::max(roi.ymax, y + (int)(1.3*expectedWidth/2));
  }
}

std::vector<ROI> ROIDetector::findROIs(){
  
  // make sure we have an image
  if (!setImagePointers())
    return vector<ROI>();

  // create a grayscale cv::Mat of the whole image, subsampled by xstep_ and ystep_
  ROI roi(0, iparams_.width, 0, iparams_.height, camera_);
  
  cv::Mat m_img = extractMat(roi);
  //cv::resize(color_segmenter_.img_grayscale(), m_img, cv::Size(), 1.0 / xstep_, 1.0 / ystep_, cv::INTER_NEAREST); 
  GaussianBlur(m_img, m_img, Size(3,3), 0, 0, BORDER_DEFAULT);
  Mat edgePoints(m_img.rows, m_img.cols, CV_8UC1, cv::Scalar(0));
  VisionTimer::Start("ROIDetector(%s)::edgepoints", camera_);
  int CONTOUR_THRESH = 10;

  // Populate the edgePoints Mat; set pixel value of edges to 255
  // A pixel is an edge if its intensity differs from its right or bottom neighbor by at least CONTOUR_THRESH
  for (int row = 0; row < m_img.rows-1; row++){
    for (int col = 0; col < m_img.cols-1; col++){
      Scalar value = m_img.at<uchar>(row, col); 
      Scalar nextValue = m_img.at<uchar>(row, col+1);
      Scalar nextRowVal = m_img.at<uchar>(row+1, col);
      int c = getSegPixelValueAt(col*xstep_, row*ystep_);  
// VERY SLOW START 
      if (c == c_FIELD_GREEN && (abs(value[0] - nextValue[0]) >= CONTOUR_THRESH || abs(value[0] - nextRowVal[0]) >= CONTOUR_THRESH)){
        edgePoints.at<uchar>(row, col) = 255;
      }
      else {
        edgePoints.at<uchar>(row, col) = 0;
      }
// VERY SLOW END    
    }   
  }
  VisionTimer::Stop("ROIDetector(%s)::edgepoints", camera_);

  const int boxThresh = 10;

  // Construct a grid of boxes. For each box, compute the bounding box of edge pixels
  int boxRows = m_img.rows/scale_;
  int boxCols = m_img.cols/scale_;
  vector<vector<ROIRect>> boxMap(boxRows, vector<ROIRect>(boxCols));
  for (int row = 0; row < boxRows; row++){
    for (int col = 0; col < boxCols; col++){
      int xmin = (col+1)*scale_ - 1;
      int xmax = col*scale_;
      int ymin = (row+1)*scale_ - 1;
      int ymax = row*scale_;
      int count = 0;

      // compute the bounding box (xmin, xmax, ymin, ymax) of the edge pixels for this box
      for (int i = 0; i < scale_; i++){
        for (int j = 0; j < scale_; j++){
          int x = col*scale_ + j;
          int y = row*scale_ + i;
          Scalar val = edgePoints.at<uchar>(y, x);
          if (val[0] > 0){
            count++;
            ymin = min(ymin, y);
            ymax = max(ymax, y);
            xmin = min(xmin, x);
            xmax = max(xmax, x);
          }
        }
      }

      // if this box has enough edge pixels, flag it as valid. Otherwise flag it as invalid
      if (count > boxThresh && xmin != xmax && ymin != ymax){
        auto& rect = boxMap[row][col];
        rect.xmin = xmin;
        rect.xmax = xmax;
        rect.ymin = ymin;
        rect.ymax = ymax;
        
        rect.boxRow = row;
        rect.boxCol = col;
        rect.valid = true;
      }
      else {
        boxMap[row][col].valid = false;
      }
    }
  }

  // Merge boxes
  Scalar colors[3] = {Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255)};
  const double SMALL_BOX_THRESH = 1; //0.75;
  for (int i = 0; i < boxRows; i++){
    for (int j = 0; j < boxCols; j++){
      ROIRect box = boxMap[i][j];
      if (!box.valid){
        continue;
      }

      // Merge the box with something else
      if (box.area() < (scale_ * scale_ * SMALL_BOX_THRESH)){
        int above_ind = max(0, i-1);
        int below_ind = min(boxRows - 1, i+1);
        int right_ind = min(boxCols - 1, j+1);
        int left_ind = max(0, j-1);
        bool is_horizontal = (box.xmax - box.xmin) > (box.ymax - box.ymin);

        if (is_horizontal){
          //Check if a box exists above
          if (above_ind != i) {
            ROIRect above_box = boxMap[above_ind][j];
            if (above_box.valid){
              mergeBoxes(boxMap[i][j], boxMap[above_ind][j]);
              continue;
            }
          }
          else if (below_ind != i) {
            ROIRect below_box = boxMap[below_ind][j];
            if (below_box.valid){
              mergeBoxes(boxMap[i][j], boxMap[below_ind][j]);
              continue;
            }
          }
        }
        else {
          if (right_ind != j){
            ROIRect right_box = boxMap[i][right_ind];
            if (right_box.valid){
              mergeBoxes(boxMap[i][j], boxMap[i][right_ind]);
              continue;
            }

          }
          else if (left_ind != j){
            ROIRect left_box = boxMap[i][left_ind];
            if (left_box.valid){
              mergeBoxes(boxMap[i][j], boxMap[i][left_ind]);
              continue;
            }
          }
        }
      }
    }
  }


/*

  // Extract lines from boxes and find the "best" parallel set
  int tmpcount = 0;
  for (int i = 0; i < boxRows; i++){
    for (int j = 0; j < boxCols; j++){
      ROIRect box = boxMap[i][j];
      if (!box.valid)
        continue;
      Rect region(box.xmin, box.ymin, box.xmax - box.xmin, box.ymax - box.ymin);

      Mat tmp(edgePoints, region);

      vector<Vec4i> lines;

      double resolution = 1; //1;      // Distance resolution in pixels
      int votesNeeded = 5; //10;
      double minLineLength = 5; //5;
      double maxLineGap = 3; //2;
      HoughLinesP(tmp, lines, resolution, CV_PI/16, votesNeeded, minLineLength, maxLineGap);
      boxMap[box.boxRow][box.boxCol].lines = lines;
      vector<Vec4i> parallellines = findLinesInBox(lines);
      boxMap[box.boxRow][box.boxCol].lineCandidate = parallellines;
    }
  }

  // Average the lines. This is mostly just for displaying, though may be necessary when passing to localization
  vector<ROIRect> allLineCands;
  for (int row = 0; row < boxRows; row++){
    for (int col = 0; col < boxCols; col++){
      ROIRect box = boxMap[row][col];
      if (box.lineCandidate.size() > 0){ //TODO - use right test here
        // Compute the line segment for this box
        Vec4i line = averageLines(box.lineCandidate[0], box.lineCandidate[1]);
        // Convert to line segment
        LineSegment lineSeg(line[0] + box.xmin , line[1] + box.ymin, line[2] + box.xmin, line[3]+box.ymin);
        box.avgLine = lineSeg;
        allLineCands.push_back(box);
      }
    }
  }

  // Recursively join boxes that contain line segments to construct contiguous lines
  vector<LineStack> fieldLines = constructLines(allLineCands);
  for (int i = 0; i < fieldLines.size(); i++){
    LineStack line = fieldLines[i];
    for (int j = 0; j < line.lineCount; j++){
      ROIRect b = allLineCands[line.lineIndices[j]];
      boxMap[b.boxRow][b.boxCol].valid = false;
    }
  }
*/

  // Return a vector of the ROIs not used to construct field lines
  vector<ROI> rois;
  for (int row = 0; row < boxRows; row++){
    for (int col = 0; col < boxCols; col++){
      ROIRect box = boxMap[row][col];
      if (!box.valid)
        continue;
      ROI roi;
      roi.xmin = box.xmin * xstep();
      roi.xmax = box.xmax * xstep();
      roi.ymin = box.ymin * ystep();
      roi.ymax = box.ymax * ystep();
      rois.push_back(roi);  
    }
  }

  toolBoxMap = boxMap;
  edgeImg = edgePoints; 

  return rois;
}

Vec4i ROIDetector::averageLines(Vec4i line1, Vec4i line2){

  Vec4i avgLine;

  LineSegment seg1(line1[0], line1[1], line1[2], line1[3]);
  LineSegment seg2(line2[0], line2[1], line2[2], line2[3]);


  float d1 = seg1.start.getDistanceTo(seg2.start);
  float d2 = seg1.start.getDistanceTo(seg2.end);

  if (d1 < d2){

    avgLine[0] = (seg1.start.x + seg2.start.x) / 2;
    avgLine[1] = (seg1.start.y + seg2.start.y) / 2;
    avgLine[2] = (seg1.end.x + seg2.end.x) / 2;
    avgLine[3] = (seg1.end.y + seg2.end.y) / 2;

  }
  else {
  
    avgLine[0] = (seg1.start.x + seg2.end.x) / 2;
    avgLine[1] = (seg1.start.y + seg2.end.y) / 2;
    avgLine[2] = (seg1.end.x + seg2.start.x) / 2;
    avgLine[3] = (seg1.end.y + seg2.start.y) / 2;
      
  }

  return avgLine;

}


void ROIDetector::recurseCheck(vector<ROIRect> &allLineCands, LineStack &linestack, int i){

  const float DIST_THRESH = 10;
  const float ANG_THRESH = 0.3;

  ROIRect boxI = allLineCands[i];

  // Check distance to subsequent lines
  for (int j = 0; j < allLineCands.size(); j++){

    ROIRect boxJ = allLineCands[j];

    // This line has probably already been used somewhere
    if (!boxJ.valid)
      continue;


    double dist = min(boxI.avgLine.getDistanceTo(boxJ.avgLine.start), boxI.avgLine.getDistanceTo(boxJ.avgLine.end));

    AngRad ang = boxI.avgLine.getSignedAngleToLine(boxJ.avgLine);


    if (dist < DIST_THRESH && abs(ang) < ANG_THRESH){

      //printf("Merged box %d with %d: (%f,%f) (%f,%f) (%f,%f) (%f,%f) \n", i, j, boxI.avgLine.start.x, boxI.avgLine.start.y, boxI.avgLine.end.x, boxI.avgLine.end.y, boxJ.avgLine.start.x, boxJ.avgLine.start.y, boxJ.avgLine.end.x, boxJ.avgLine.end.y);

      // Merge
      linestack.lineIndices[linestack.lineCount++] = j;

      // Mark as used.
      allLineCands[j].valid = false;

      // Recurse check for more
      recurseCheck(allLineCands, linestack, j);

    }

  }


}

vector<LineStack> ROIDetector::constructLines(vector<ROIRect> &allLineCands){


  vector<LineStack> fieldLines;

  Scalar colors[3] = {Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255)};
  int stackCount = 0;

  for (int i = 0; i < allLineCands.size(); i++){

    if (!allLineCands[i].valid)
      continue;

    ROIRect boxI = allLineCands[i];
    
    allLineCands[i].valid = false;      // Mark as used

    LineStack linestack;
    linestack.lineIndices[0] = i;
    linestack.lineCount = 1;

      
    recurseCheck(allLineCands, linestack, i);
   stackCount++;

  // TODO: Instead of using the count, we should check the actual size of the line. 
    if (linestack.lineCount > 2){
      fieldLines.push_back(linestack);
    }

  }

  
  return fieldLines;
}



void ROIDetector::mergeBoxes(ROIRect &source, ROIRect &target){
  target.xmin = min(target.xmin, source.xmin);
  target.ymin = min(target.ymin, source.ymin);
  target.xmax = max(target.xmax, source.xmax);
  target.ymax = max(target.ymax, source.ymax);
  source.valid = false;
}


vector<Vec4i> ROIDetector::findLinesInBox(vector<Vec4i> linesInBox){

  //printf("Finding lines in box\n");

  vector<Vec4i> foundLines;
  const float ANGLE_DEV_THRESH = 0.35;//in radians
  const float DIST_DEV_THRESH = 3; 
  
  
  vector<LineSegment> lines;
  

  // Check for slope deviations
  vector<double> angles;
  double mean = 0;

  for (int i = 0; i < linesInBox.size(); i++){
    Vec4i points = linesInBox[i];
    LineSegment line(points[0], points[1], points[2], points[3]);
    lines.push_back(line);
    double ang = atan2(-line.m_b, line.m_a);
    angles.push_back(ang);
    mean += ang;
  }

  mean /= angles.size();

  double std = 0;
  for (int i = 0; i < angles.size(); i++){
    std += pow(angles[i] - mean, 2);
  }
  std = sqrt(std);

  int bestI = -1;
  int bestJ = -1;
  double minDev = 100000;

  for (int i = 0; i < lines.size(); i++){
    for (int j = i+1; j < lines.size(); j++){
      AngRad ang_is = abs(lines[i].getAngleToLine(lines[j]));

      // Parallel
      if (ang_is <= ANGLE_DEV_THRESH){
          float dist = lines[i].getDistanceTo(lines[j].start, lines[j].end);
          if (dist > DIST_DEV_THRESH && ang_is < minDev){
            
            bestI = i;
            bestJ = j;
            minDev = ang_is;
          }
      }
    }
  }

  if (bestI > -1 && bestJ > -1){
    foundLines.push_back(linesInBox[bestI]);
    foundLines.push_back(linesInBox[bestJ]);
    //printf("Found pair\n");
  }

  return foundLines;//returns empty if none found.
}

/*
 * Copies a region of the raw image to a grayscale cv::Mat
 */
cv::Mat ROIDetector::extractMat(const ROI& roi, bool highres) const {
  int xstep = highres ? 1 : xstep_;
  int ystep = highres ? 1 : ystep_;

  return color::rawToMatGraySubset(img_, iparams_, roi.ymin, roi.xmin, (roi.xmax - roi.xmin), (roi.ymax - roi.ymin), xstep, ystep);
}

cv::Mat ROIDetector::extractMat(const ROI& roi, int scaleFactor) const {
  return color::rawToMatGraySubset(img_, iparams_, roi.ymin, roi.xmin, (roi.xmax - roi.xmin), (roi.ymax - roi.ymin), scaleFactor, scaleFactor);
}
