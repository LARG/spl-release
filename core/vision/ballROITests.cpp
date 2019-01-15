// g++ /home/jkelle/nao/trunk/core/vision/ballROITests.cpp -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_objdetect -std=c++0x -o ballROITests

#include <iostream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define DEBUG_TIMING false


struct EuclideanDistanceFunctor
{
    int _dist2;
    EuclideanDistanceFunctor(int dist) : _dist2(dist*dist) {}

    bool operator()(const cv::Point& lhs, const cv::Point& rhs) const
    {
        return ((lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y)) < _dist2;
    }
};


struct ROI {
  ROI() { }
  ROI(int xmin, int xmax, int ymin, int ymax) : xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax) { }

  bool operator==(const ROI& other) const { 
    return xmin == other.xmin && xmax == other.xmax && ymin == other.ymin && ymax == other.ymax;
  }

  int xmin, xmax, ymin, ymax;
};


int main(int argc, char* argv[]) {
  int fileIndex = atoi(argv[1]);

  int start_time_func = cv::getTickCount();
  int ticksForExtractMat = 0;
  int ticksForAdaptiveThreshold = 0;
  int ticksForFindingDarks = 0;
  int ticksForClustering = 0;
  int ticksForMakingROI = 0;
  int ticksForGrowing = 0;

  // make sure we have an image
  // if (!setImagePointers())
  //   return vector<ROI>();

  int xstep = 8;
  int ystep = 4;
  // if(camera_ == ) {
    // int xstep = 2;
    // int ystep = 2;
  // }

  char buff[100];
  snprintf(buff, sizeof(buff), "/home/jkelle/nao/trunk/top_%02d.png", fileIndex);
  std::string filepath = buff;

  // create a grayscale cv::Mat of the whole image, subsampled by xstep_ and ystep_
  int start_time = cv::getTickCount();
  cv::Mat mat = cv::imread(filepath, CV_LOAD_IMAGE_GRAYSCALE);
  // cv::imwrite("loadedImage.png", mat);
  ticksForExtractMat += cv::getTickCount() - start_time;

  // adaptive threshold image with high threshold
  start_time = cv::getTickCount();
  cv::Mat binaryImg;
  // for(int window = 3; window < 20; window+=2) {
    // for(int t=5; t < 150; t+=5) {
      int window = 5;
      int t = 70;
      cv::adaptiveThreshold(mat, binaryImg, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, window, t);
      std::cout << "done with adaptiveThreshold()" << std::endl;
      snprintf(buff, sizeof(buff), "/home/jkelle/nao/trunk/adaptive_%02d_%02d_%03d.png", fileIndex, window, t);
      filepath = buff;
      cv::imwrite(filepath, binaryImg);
    // }
  // }
  ticksForAdaptiveThreshold += cv::getTickCount() - start_time;

  // find dark pixels
  start_time = cv::getTickCount();
  std::vector<cv::Point> points;
  for(int row = 0; row < binaryImg.rows; ++row) {
    for(int col = 0; col < binaryImg.cols; ++ col) {
      if(binaryImg.at<unsigned char>(row, col) < 255) {
        points.push_back(cv::Point(col, row));
      }
    }
  }
  ticksForFindingDarks += cv::getTickCount() - start_time;

  // cluster pixels
  for(int d = 6; d <= 16; d+= 2) {
    start_time = cv::getTickCount();
    std::vector<int> labels;
    int n_labels = cv::partition(points, labels, EuclideanDistanceFunctor(d));
    ticksForClustering += cv::getTickCount() - start_time;

    // make ROI objects from clustered pixels
    start_time = cv::getTickCount();
    std::vector<ROI> rois(n_labels, ROI(10000, -1, 100000, 1));
    for(int i = 0; i < labels.size(); ++i) {
      rois[labels[i]].xmin = std::min(rois[labels[i]].xmin, points[i].x);
      rois[labels[i]].xmax = std::max(rois[labels[i]].xmax, points[i].x);
      rois[labels[i]].ymin = std::min(rois[labels[i]].ymin, points[i].y);
      rois[labels[i]].ymax = std::max(rois[labels[i]].ymax, points[i].y);
    }
    ticksForMakingROI += cv::getTickCount() - start_time;

    std::vector<ROI> filteredROIs;
    for(auto roi : rois) {
      int width = roi.xmax - roi.xmin;
      int height = roi.ymax - roi.ymin;
      std::cout << width << " " << height << std::endl;
      if (width > 1 && height > 1) {
        filteredROIs.push_back(roi);
      }
    }
   
    cv::Mat colorMat;
    cv::cvtColor(mat, colorMat, CV_GRAY2BGR);
    cv::Vec3b colorRed(0,0,255);
    for(auto roi : filteredROIs) {
      cv::rectangle(colorMat, cv::Point(roi.xmin, roi.ymin), cv::Point(roi.xmax, roi.ymax), cv::Scalar(colorRed), 1);
    }

    snprintf(buff, sizeof(buff), "/home/jkelle/nao/trunk/rois_%02d_%02d.png", fileIndex, d);
    filepath = buff;
    cv::imwrite(filepath, colorMat);

    cv::Mat colorMat2;
    cv::cvtColor(binaryImg, colorMat2, CV_GRAY2BGR);
    for(auto roi : filteredROIs) {
      cv::rectangle(colorMat2, cv::Point(roi.xmin, roi.ymin), cv::Point(roi.xmax, roi.ymax), cv::Scalar(colorRed), 1);
    }
    snprintf(buff, sizeof(buff), "/home/jkelle/nao/trunk/rois_%02d_%02d_binary_.png", fileIndex, d);
    filepath = buff;
    cv::imwrite(filepath, colorMat2);
  }
  // make roi bounding boxes larger, and scale them up to highres cooridinates
  // start_time = cv::getTickCount();
  // for(int i = 0; i < rois.size(); ++i) {
  //   // scale the cooridinates to original resolution
  //   int xmin = rois[i].xmin*xstep;
  //   int ymin = rois[i].ymin*ystep;
  //   int xmax = rois[i].xmax*xstep;
  //   int ymax = rois[i].ymax*ystep;

  //   // grow the bounding box more when the ball is closer to the robot => when y is larger
  //   float paddingFactor = (1+(float(ymin))/mat.cols);
  //   int xpadding = int(paddingFactor*30);
  //   int ypadding = int(paddingFactor*30);
  //   xmin -= xpadding/2;
  //   ymin -= ypadding/2;
  //   xmax += xpadding/2;
  //   ymax += ypadding/2;

  //   // shift the ROI down (increase y) becuase it has a tendancy to be a little too high
  //   // This is because the black marks on the top of the ball get detected while the
  //   // black marks on the bottom of the ball don't.
  //   ymin += 10;
  //   ymax += 10;

  //   // clip the bounding box to the image extent
  //   rois[i].xmin = std::max(xmin, 0);
  //   rois[i].ymin = std::max(ymin, 0);
  //   rois[i].xmax = std::min(xmax, mat.cols*xstep);
  //   rois[i].ymax = std::min(ymax, mat.rows*ystep);
  // }
  // ticksForGrowing += cv::getTickCount() - start_time;

  // if(DEBUG_TIMING) {
  //   int end_time = cv::getTickCount();
  //   std::cout << "TIMING \t\t findBallROIs " << (end_time - start_time_func)/cv::getTickFrequency()*1000.0 << std::endl;
  //   std::cout << "TIMING \t\t\t extractMat " << ticksForExtractMat/cv::getTickFrequency()*1000 << std::endl;
  //   std::cout << "TIMING \t\t\t adaptiveThreshold " << ticksForAdaptiveThreshold/cv::getTickFrequency()*1000 << std::endl;
  //   std::cout << "TIMING \t\t\t collect_black_pixels " << ticksForFindingDarks/cv::getTickFrequency()*1000 << std::endl;
  //   std::cout << "TIMING \t\t\t partition " << ticksForClustering/cv::getTickFrequency()*1000 << std::endl;
  //   std::cout << "TIMING \t\t\t makeROI " << ticksForMakingROI/cv::getTickFrequency()*1000 << std::endl;
  //   std::cout << "TIMING \t\t\t growROI " << ticksForGrowing/cv::getTickFrequency()*1000 << std::endl;
  // }


  return 0;
}