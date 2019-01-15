// #pragma once

// #include <vision/Constants.h>
// #include <common/Field.h>
// #include <vision/ObjectDetector.h>
// #include <vision/ColorSegmenter.h>
// #include <vision/estimators/BallEstimator.h>
// #include <vision/structures/BallCandidate.h>
// #include <vision/structures/HorizonLine.h>
// #include <vision/structures/ROI.h>
// #include <vision/Macros.h>
// #include <math/Point.h>
// #include <common/Profiling.h>
// #include <vision/BlobDetector.h>
// #include <vision/estimators/MovingBallEstimator.h>

// class Classifier;
// class ROIDetector;

// // struct ScanLine {
// //   unsigned char color;
// //   int pixelIntensitySum;
// //   int lightestPixel = 0;
// //   int start;
// //   int end;
// //   int y;
// //   ScanLine* parent;
// //   std::vector<ScanLine*> children;
// // };

// // struct BallBlob {
// //   std::vector<ScanLine> scanLinesVector;
// //   unsigned char color;
// //   int minX, minY, maxX, maxY;
// //   int id = -1;
// //   int area = 0; // number of pixels
// //   int width, height;
// //   int bboxCenterX, bboxCenterY;
// //   float centroidX = 0.0;
// //   float centroidY = 0.0;
// //   int avgPixelIntensity = 0;
// //   int lightestPixel = 0;
// //   bool was_merged = false;

// //   BallBlob() { }
  
// //   BallBlob(ScanLine* root) {
// //     color = root->color;
// //     scanLinesVector.push_back(*root);
// //     minX = root->start;
// //     minY = root->y;
// //     maxX = root->end + 1;
// //     maxY = minY + 1;
// //     avgPixelIntensity += root->pixelIntensitySum;
// //     area += root->end - root->start + 1;
// //     centroidX += (root->end + root->start)*(root->end - root->start + 1)/2.0;
// //     centroidY += (root->y)*(root->end - root->start + 1);
    
// //     ScanLine* sl;
// //     for (auto it = root->children.begin(); it != root->children.end(); ++it) {
// //       sl = *it;
// //       scanLinesVector.push_back(*sl);
// //       minX = std::min(minX, sl->start);
// //       minY = std::min(minY, sl->y);
// //       maxX = std::max(maxX, sl->end+1);
// //       maxY = std::max(maxY, sl->y+1);
// //       avgPixelIntensity += sl->pixelIntensitySum;
// //       lightestPixel = std::max(lightestPixel, sl->lightestPixel);
// //       area += sl->end - sl->start + 1;
// //       centroidX += (sl->end + sl->start)*(sl->end - sl->start + 1)/2.0;
// //       centroidY += (sl->y)*(sl->end - sl->start + 1);
// //     }

// //     width = maxX - minX;
// //     height = maxY - minY;
// //     bboxCenterX = (maxX + minX)/2;
// //     bboxCenterY = (maxY + minY)/2;
// //     if(area > 0) {
// //       avgPixelIntensity /= area;
// //       centroidX /= area;
// //       centroidY /= area;
// //     }

// //     for (auto& sl : scanLinesVector) {
// //       assert(sl.start >= 0 && sl.end <= 1280 && sl.y >= 0 && sl.y <= 960);
// //     }
// //   }

// //   std::vector<ScanLine> getScanLineVector() const {
// //     return scanLinesVector;
// //   }

// //   // put larger blobs first when calling sort() on an collection of BallBlob objects
// //   bool operator < (const BallBlob& other) const {
// //     return area > other.area;
// //   }

// //   float getAvgIntensityWithoutLightest() const {
// //     return (avgPixelIntensity*area - lightestPixel)/(area - 1.0);
// //   }

// //   void merge(BallBlob& other, int num_pixels_between) {
// //     minX = std::min(minX, other.minX);
// //     minY = std::min(minY, other.minY);
// //     maxX = std::max(maxX, other.maxX);
// //     maxY = std::max(maxY, other.maxY);

// //     width = maxX - minX;
// //     height = maxY - minY;
// //     bboxCenterX = (maxX + minX)/2;
// //     bboxCenterY = (maxY + minY)/2;

// //     avgPixelIntensity = (avgPixelIntensity*area + other.avgPixelIntensity*other.area)/(area + other.area);
// //     lightestPixel = std::max(lightestPixel, other.lightestPixel);

// //     centroidX = (centroidX*area + other.centroidX*other.area)/(area + other.area);
// //     centroidY = (centroidY*area + other.centroidY*other.area)/(area + other.area);

// //     area += other.area + num_pixels_between;

// //     // add the scanlines from the other blob to this one.
// //     scanLinesVector.insert(scanLinesVector.end(), other.scanLinesVector.begin(), other.scanLinesVector.end());

// //     was_merged = true;
// //   }

// //   bool wasMerged() {
// //     return was_merged;
// //   }

// // };

// // class BlobTriangle {
// //   public:
// //     BlobTriangle(float score = std::numeric_limits<float>::lowest()) : score(score) { }
// //     BlobTriangle(const BallBlob& b1, const BallBlob& b2, const BallBlob& b3) : b1(b1), b2(b2), b3(b3) { }
// //     BallBlob b1, b2, b3;
// //     float centerX, centerY, radius, score;
// // };

// /// @ingroup vision
// class BallDetector_deepnet : public ObjectDetector {
//  public:
//   BallDetector_deepnet(DETECTOR_DECLARE_ARGS, const ROIDetector& roi_detector, ColorSegmenter& segmenter, BlobDetector& blob_detector);
//   ~BallDetector_deepnet();
//   void init(TextLogger* tl) final;
//   void findBall(std::vector<ROI>& rois);
//   inline void setHorizon(HorizonLine horizon) { horizon_ = horizon; }
//   inline std::vector<BallCandidate> candidates() const { return candidates_; }
//   inline BallCandidate* best() const { return best_.get(); }
 
//  private:
//   float getDirectDistanceByBlobWidth(float width, int centerX, int centerY);
//   float getDirectDistanceByKinematics(int x, int y);
//   float getDirectDistance(BallCandidate& candidate);
//   bool setBest(BallCandidate& candidate);
//   // BlobTriangle blobTests(cv::Mat& mat, const ROI& roi, int xstep, int ystep, bool highres);
//   // BlobTriangle blobGeometryTests(std::vector<BlobTriangle>& triangles, const std::vector<BallBlob>& blobs, const ROI& roi, int xstep, int ystep);
//   // BlobTriangle blobGeometryTests(BlobTriangle& triangle, const std::vector<BallBlob>& blobs, const ROI& roi, int xstep, int ystep);


//   // Moving ball using blobs
//   bool findMovingBall();
//   BallCandidate formBallCandidateFromBlob(Blob* blob);
//   void fitCircleToPoints(uint16_t *x, uint16_t *y, uint16_t n, float *cx, float *cy, float *radius, float *stddev);
//   void getBlobContour(Blob* blob, uint16_t *xinit, uint16_t *xfinal);
//   BallCandidate* evaluateMovingBallCandidates(std::vector<BallCandidate>& candidates);
//   bool intersectsShoulder(BallCandidate* candidate);
//   void checkColorsInCircleFit(BallCandidate* candidate, float& whitePct, float& undefPct, float& pctBall);
//   bool checkSurroundedByGreen(BallCandidate& candidate);
//   float greenInBox(int xmin, int xmax, int ymin, int ymax);
  
//   // for dubugging
//   int counter_ = 0;
//   int roiCounter_ = 0;

//   // blob formation
//   // std::vector<BallBlob> computeBlobs(cv::Mat& mat, std::vector<std::vector<ScanLine>>& lines);
//   // void merge(ScanLine& top, ScanLine& bot);
//   // ScanLine* getRoot(ScanLine& sl);
//   // bool doesChangeColor(cv::Mat& mat, int row, int col, int window, int& left, int& right);
//   // std::vector<BallBlob> filterBlobs(std::vector<BallBlob> blobs, std::vector<BallBlob>& badBlobs, const ROI& roi, int xstep, int ystep, bool highres);
//   // int greenPixelCount(const BallBlob& blob, const ROI& roi, int xstep, int ystep);
//   // bool blobTooBig(BallBlob& blob, const ROI& roi);
//   // bool blobTooSmall(BallBlob& blob, const ROI& roi);
//   // float getExpectedBlobCameraWidth(BallBlob& blob);
//   // void refineBallLocation(BallCandidate& candidate, const ROI* roi, cv::Mat& mat, BlobTriangle triangle);
//   // void eraseBlob(cv::Mat& mat, BallBlob blob);
//   // bool shouldMergeBlobs(BallBlob& b1, BallBlob& b2);
  
//   // for debugging
//   // void createBlobImage(cv::Mat& mat, std::vector<BallBlob> goodBlobs, std::vector<BallBlob> badBlobs, std::string filepath);
//   // void colorBlob(cv::Mat& seg, BallBlob blob, cv::Vec3b color);

//   // float checkBelowGreenPct(BallCandidate &candidate, bool useWhite);
//   // float checkBallColor(BallCandidate &candidate);

//   // float max(float a, float b, float c);
//   // float min(float a, float b, float c);

//   std::vector<BallCandidate> candidates_;
//   std::unique_ptr<BallCandidate> best_;
//   std::unique_ptr<Classifier> classifier_;
//   HorizonLine horizon_;
//   const ROIDetector& roi_detector_;
//   ColorSegmenter& color_segmenter_;
//   BlobDetector & blob_detector_;

//   // BallEstimator estimator_;

//   MovingBallEstimator movingball_estimator_;
  
// };
