//#include <vision/YoloBallDetector.h>
#include <vision/ROIDetector.h>
#include <vision/ColorSegmenter.h>
#include <vision/ml/SvmClassifier.h>
// #include <vision/ml/DeepClassifier.h>
#include <VisionCore.h>
#include "YoloBallDetector.h"
#include <common/Field.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

#include <read_detect_lib_config.h>

#define NUM_THREADS 4

using CrossCandidate = BallCandidate;

double get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }


#define TFLITE_MINIMAL_CHECK(x)                              \
      if (!(x))                                                  \
  {                                                          \
          fprintf(stderr, "Error at %s:%d\n", __FILE__, __LINE__); \
          exit(1);                                                 \
        }

#define horizontalBlob blob_detector_.horizontalBlob
#define getball() vblocks_.world_object->objects_[WO_BALL]
#define getcross() vblocks_.world_object->objects_[WO_UNKNOWN_PENALTY_CROSS]
#define getself() vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF]
#define getframe() vblocks_.frame_info->frame_id

#define DARK 0
#define WHITE 1

#define DEBUG_BLOB false
#define DEBUG_TEST false
#define DEBUG_TIMING false
#define DEBUG_WRITE_ROIS false

#define LOG_NEGATIVES false

using namespace Eigen;
using namespace cv;
using namespace std;


void DrawRectangles(
  cv::Mat & img,
  const vector<vector<float>> & vecVecFloat
) {
  for (const auto & vec: vecVecFloat) {
    cv::Rect rect = cv::Rect(cv::Point(vec[0], vec[1]), cv::Point(vec[2], vec[3]));
    cv::rectangle(img, rect,  cv::Scalar(0, 0, 255), 1);
  }
}


bool YoloBallDetector::setImagePointers() {
  if(vblocks_.image == NULL) {
    printf("No image block loaded!\n");
    return false;
  }
  if(camera_ == Camera::TOP) {
    img_ = vblocks_.image->getImgTop();
    detector_ = top_detector_;
    if(!img_) return false;
  }
  else {
    img_ = vblocks_.image->getImgBottom();
    detector_ = bottom_detector_;
    if(!img_) return false;
  }
  return true;
}


YoloBallDetector::YoloBallDetector(DETECTOR_DECLARE_ARGS, FieldEdgeDetector& field_edge_detector) : DETECTOR_INITIALIZE, field_edge_detector_(field_edge_detector) {
}

YoloBallDetector::~YoloBallDetector() {
}

// Ishan TODO: check if textlogger is used here
// Ishan: TODO add second classifier for bottom camera
// in setImagePointers() set the pointers to the corresponding classifier and it's respective sizes
void YoloBallDetector::init(TextLogger* tl) {
  // Proof of concept to create a detector object
  #ifdef TOOL
  std::string nao_home_path(getenv("NAO_HOME"));
  std::string yaml_path_top = nao_home_path + "/lib/detect_lib/testing/models/top_tool.yaml";
  std::string yaml_path_bottom = nao_home_path + "/lib/detect_lib/testing/models/bottom_tool.yaml";
  #else
  std::string yaml_path_top("data/models/top.yaml");
  std::string yaml_path_bottom("data/models/bottom.yaml");
  #endif
  top_detector_config = yaml_to_config(&yaml_path_top[0]);
  bottom_detector_config = yaml_to_config(&yaml_path_bottom[0]);
  #ifdef TOOL
  top_detector_config->model_path = nao_home_path + top_detector_config->model_path;
  bottom_detector_config->model_path = nao_home_path + bottom_detector_config->model_path;
  #endif
  if(camera_ == Camera::TOP)
  {
    top_detector_ = new TFLiteDetector(top_detector_config);
  }
  else
  {
    bottom_detector_ = new TFLiteDetector(bottom_detector_config);
  }
}

void YoloBallDetector::findBall() {
  // set image pointer and check if image loaded
  if (!setImagePointers())
      return;

  // std::cerr << "Frame started YOLO" << endl;
  
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  VisionTimer::Start(70, "BallDetector(%s)::findBall", camera_);
  best_.reset();

  //if (camera_ == Camera::BOTTOM)
  CrossCandidate best_cc;
  BallCandidate best_c;
  {
    candidates_.clear();
    cross_candidates_.clear();
    robot_candidates_.clear();
  }

#ifdef TOOL
  blobDebugImg = cv::Mat(iparams_.height, iparams_.width, CV_8UC3);
  blobDebugImg.setTo(cv::Scalar(255,255,255));
#endif

  WorldObject &ball = getball();
  // commenting out the section below because we still want to detect the robots if they are there
  /*// if ball was already detected by bottom camera, and we're in top cam, done
  if(ball.seen && !ball.fromTopCamera && camera_ == Camera::TOP) {
    tlog(70, "Ball was seen already in the bottom camera, bailing out.");
    VisionTimer::Stop("BallDetector(%s)::findBall", camera_);
    counter_++;     // So that debug_blob images don't overwrite between top and bottom cam
    return;
  }*/

  if(DEBUG_WRITE_ROIS && camera_ != Camera::TOP) {
    VisionTimer::Stop("BallDetector(%s)::findBall", camera_);
    return;
  }

  // Ishan TODO implement the function to read image into matrix
  // cv::Mat cvimg = color::rawToMat(img_, iparams_); //= read_img(img_);
  // tlog(70, "image read into matrix");
  // cv::cvtColor(cvimg, cvimg, CV_RGB2BGR);
  // cv::imwrite("logs/converted.png", cvimg);
  // cv::cvtColor(cvimg, cvimg, CV_BGR2RGB);

// #if (CV_VERSION_MAJOR >= 4)
//     cv::cvtColor(cvimg, cvimg, COLOR_BGR2RGB);
// #else
//       cv::cvtColor(cvimg, cvimg, CV_BGR2RGB);
// #endif
  vector<vector<float>> rects;
  vector<float> box;
  vector<float> box_cross;

  /*if (detector_ == top_detector_)
  {
    // cv::resize(cvimg, cvimg, Size(width_top, height_top));
    box = detector_->DetectRGB(cvimg);
  }
  else
  {
    // cv::resize(cvimg, cvimg, Size(width_bottom, height_bottom));
    box = detector_->DetectRGB(cvimg);
  }*/
  
  // Call detector directly with image pointer
  begin = std::chrono::steady_clock::now();
  if((camera_ == Camera::TOP) || (camera_ == Camera::BOTTOM)) {
    rects = detector_->DetectMulticlass(img_);
  }
  // else{
  //   rects.push_back(detector_->DetectDirect(img_));
  // }
  end = std::chrono::steady_clock::now();

  // for(auto it=box.begin(); it !=box.end(); ++it)
  // {
  //   std::cerr << *it << std::endl;
  // }

  int bestCandId = -1; 
  int bestCrossCandId = -1; 

  if(!rects.empty()) { 
    // float curBestScore = 0.;
    float scale = 0.5;
    DetectLibConfig * cur_config;
    if(true) {
      if(camera_ == Camera::TOP)
        cur_config = top_detector_config;
      else
        cur_config = bottom_detector_config;
      float maxconf = 0;
      float maxconf_cross = 0;
      scale = 0.5*(cur_config->subsample_major + cur_config->subsample_minor);
      for(auto & rect : rects)
      {
         
        if (((int)rect[5])==0 && rect[4]>=maxconf)
        {
          auto objX = scale * rect[0];
          auto objY = scale * rect[1];
          auto& self = getself();
          Position p_cand = cmatrix_.getWorldPosition(objX, objY, BALL_RADIUS);
          Point2D cand_world = Point2D(p_cand.x, p_cand.y).relativeToGlobal(self.loc, self.orientation);
          if(abs(cand_world.x) > (HALF_FIELD_X + BORDER_STRIP_WIDTH) || abs(cand_world.y) > (HALF_FIELD_Y + BORDER_STRIP_WIDTH))
              continue;
          /// Ignore candidate if the bottom is above the horizon
          if (horizon_.isAbovePoint( scale * (rect[0] ), scale* (rect[1] + rect[3] * 0.5))) {
            //  || horizon_.isAbovePoint( scale * (rect[0] - rect[2] / 2), scale * (rect[1] + rect[3] * 0.5))) {
            scale = 0.5*(cur_config->subsample_major + cur_config->subsample_minor);
            best_c.confidence = rect[4];
            best_c.centerX = scale * rect[0];
            best_c.centerY = scale * rect[1];
            best_c.width = scale * rect[2];
            best_c.height = scale * rect[3];
            best_c.radius = (best_c.width + best_c.height) / 4.0;
            // getDirectDistance uses both kinematics and ball width to estimate relative distance
            float d_ = getDirectDistance(best_c);

            // hand tuning these confidences for the thailand quarter finals but
            // should be part of detect lib
            // the we want to be more strict on the ball than cross
            float min_conf = 0.0;
            if(camera_ == Camera::TOP) {
              min_conf = 0.9 - 0.2 * fmax(d_, 3000) / 3000;
            } else {
              min_conf = 0.0;
            }

            if (rect[4] > min_conf) {
              box = rect;
              maxconf = rect[4];
            }
          }
        }
        if (((int)rect[5])==1 && rect[4]>=maxconf_cross)
        {
          /// Ignore candidate if the bottom is above the horizon
          if (horizon_.isAbovePoint( scale * (rect[0] ), scale* (rect[1] + rect[3] * 0.5))) {
            scale = 0.5*(cur_config->subsample_major + cur_config->subsample_minor);
            box = rect;
            best_cc.confidence = box[4];
            best_cc.centerX = scale * box[0];
            best_cc.centerY = scale * box[1];
            best_cc.width = scale * box[2];
            best_cc.height = scale * box[3];
            best_cc.radius = (best_cc.width + best_cc.height) / 4.0;
            // cout << "======= NN DETECTING CROSS with confidence " << best_cc.confidence << " ==========" << endl;
            Position p = cmatrix_.getWorldPosition(best_cc.centerX, best_cc.centerY, PENALTY_MARK_SIZE);
            best_cc.groundDistance = cmatrix_.directDistance(p);
            if (best_cc.groundDistance < 4000) {
              box_cross = box;
              maxconf_cross = box[4];
              best_cc.relPosition = cmatrix_.getWorldPositionByDirectDistance(best_cc.centerX, best_cc.centerY, best_cc.groundDistance);
              auto& self = getself();
              Point2D cand = Point2D(best_cc.relPosition.x, best_cc.relPosition.y).relativeToGlobal(self.loc, self.orientation);
              best_cc.absPosition = Position(cand.x, cand.y, 0);
              // fprintf(stdout, "======= NN DETECTING CROSS at distance %.2f with confidence %.2f at rel-pos (%.1f,%.1f) and abs-pol (%.1f,%.1f)\n", best_cc.groundDistance, best_cc.confidence, best_cc.relPosition.x, best_cc.relPosition.y, cand.x, cand.y);
            }
          }
        }
        if (((int)rect[5]) == 2)
        {
            RobotCandidate candidate;
            candidate.xi = scale *(rect[0] - (0.5*rect[2]));
            candidate.xf = scale *(rect[0] + (0.5*rect[2]));
            candidate.yi = scale *(rect[1] - (0.5*rect[3]));
            candidate.yf = scale *(rect[1] + (0.5*rect[3]));

            candidate.avgX = scale * rect[0];
            candidate.avgY = scale * rect[1];

            candidate.width = scale * rect[2];
            candidate.height = scale * rect[3];

            // candidate.xi = scale *(rect[0]);
            // candidate.xf = scale *(rect[2]);
            // candidate.yi = scale *(rect[1]);
            // candidate.yf = scale *(rect[3]);

            // candidate.avgX = scale * 0.5 * (rect[0] + rect[2]);
            // candidate.avgY = scale * 0.5 * (rect[1] + rect[3]);

            // candidate.width = scale * abs(rect[0] - rect[2]);
            // candidate.height = scale * abs(rect[1] - rect[3]);

            candidate.relPosition = cmatrix_.getWorldPosition(candidate.avgX, candidate.yf);
            Point2D relPos(candidate.relPosition.x, candidate.relPosition.y);
            Point2D globPos = relPos.relativeToGlobal(getself().loc, getself().orientation);
            candidate.absPosition = Position(globPos.x, globPos.y, 0); 

            candidate.confidence = rect[4];

            robot_candidates_.push_back(candidate);
        }
      }
    }
    else{
      cur_config = bottom_detector_config;
      box = rects[0];
    }

    if(!box_cross.empty()) {
      bestCrossCandId = 0;
      best_cc.valid = true;
      cross_candidates_.push_back(best_cc);
    }

    if(box.empty()) {
        goto jmp;
    } 

    best_c.confidence = box[4];
    float directDistance = getDirectDistance(best_c);
    best_c.relPosition = cmatrix_.getWorldPositionByDirectDistance(best_c.centerX, best_c.centerY, directDistance);
    best_c.groundDistance = cmatrix_.groundDistance(best_c.relPosition);
    best_c.relPosition.z -= BALL_RADIUS;
    best_c.valid = true;
    auto& self = getself();
    Point2D cand = Point2D(best_c.relPosition.x, best_c.relPosition.y).relativeToGlobal(self.loc, self.orientation);
    best_c.absPosition = Position(cand.x, cand.y, 0);
    float cross_dist = 0.0;
    // if(vblocks_.robot_state->WO_SELF == WO_TEAM1)
    // {
    //   float cross_dist = (cand - ownCrossLocation).getMagnitude();
    //   if(cross_dist < 300)
    //     goto jmp;
    // }


    //float scale = 5.0;
    /*if(camera_ != Camera::TOP) {
      fprintf(stderr, "Bottom camera: ");
    }
    else{
       fprintf(stderr, "Top camera: ");
       scale= 2.5;
    }
    fprintf(stdout, "Found ball with conf %.2f\n", c.confidence);
    fprintf(stdout, "   with coords  %.2f,%.2f\n", c.centerX, c.centerY);
    
    vector<vector<float>> rectangles = {
      {
        scale*(box[0] - 0.5f * box[2]),
        scale*(box[1] - 0.5f * box[3]),
        scale*(box[0] + 0.5f * box[2]),
        scale*(box[1] + 0.5f * box[3]),
      }
    };
    cv::Mat cvimg = color::rawToMat(img_, iparams_); //= read_img(img_);
    tlog(70, "image read into matrix");
    cv::cvtColor(cvimg, cvimg, CV_RGB2BGR);
    DrawRectangles(cvimg, rectangles);
    cv::putText(cvimg, 
            "Here is some text",
            cv::Point(rectangles[0][0],
                      rectangles[0][1]), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(255,255,255), // BGR Color
            1 // Line Thickness (Optional)
            ); // Anti-alias (Optional)
    if(camera_ == Camera::TOP) {
      cv::imwrite("logs/converted.png", cvimg);
    }*/
    

    // get ball velocity
    float dist = (ball.loc - cand).getMagnitude();
    int frames = getframe() - ball.frameLastSeen;
    if (frames > 0)
        best_c.velocity = dist / frames;
    else
        best_c.velocity = 0;
#ifdef TOOL
    best_c.velocity = 0; // Velocity can be problematic when running logs, especially when log frames aren't sequential
#endif

    // // find best ball estimate
    // if(conf > curBestScore) {
    //     curBestScore = conf;
    //     bestCandId = i;
    // }
    bestCandId = 0;
    candidates_.push_back(best_c);
  }
  else {
    jmp:
    VisionTimer::Stop("BallDetector(%s)::findBall", camera_);
    return;
  }
  
  // set best ball
  if (bestCandId != -1) {
    setBest(candidates_[bestCandId]);
  }

  if (bestCrossCandId != -1) {
    setCrossBest(cross_candidates_[bestCrossCandId]);
  }

  selectRobots(robot_candidates_);


/*
  vector<vector<float>> boxes;
  vector<float> confidence;
  for (int i=0; i < height; ++i) {
    for (int j=0; j < width; ++j) {
      auto &rgb = cvimg.at<cv::Vec3b>(i, j);
      interpreter->typed_input_tensor<float>(0)[3 * (i * width + j) + 0]
         = float(rgb[0]) / 255.0f;
      interpreter->typed_input_tensor<float>(0)[3 * (i * width + j) + 1]
         = float(rgb[1]) / 255.0f;
      interpreter->typed_input_tensor<float>(0)[3 * (i * width + j) + 2]
         = float(rgb[2]) / 255.0f;
    }
  }
  // Run inference
  TFLITE_MINIMAL_CHECK(interpreter->Invoke() == kTfLiteOk);
 
  float curBestScore = 0.;
 int bestCandId = -1; 
  // Read output tensor and add detections above confidence threshold to vector
  for (auto i = 0; i < num_output_boxes; ++i) {
    float conf = interpreter->typed_output_tensor<float>(0)[(5 + num_classes) * i + 4];
    if (conf < min_confidence)
      continue;
    BallCandidate c;
    c.confidence = conf;
    c.centerX = interpreter->typed_output_tensor<float>(0)[(5 + num_classes) * i + 0];
    c.centerY = interpreter->typed_output_tensor<float>(0)[(5 + num_classes) * i + 1];
    c.width = interpreter->typed_output_tensor<float>(0)[(5 + num_classes) * i + 2];
    c.height = interpreter->typed_output_tensor<float>(0)[(5 + num_classes) * i + 3];
    c.radius = (c.width + c.height) / 2;
    float directDistance = getDirectDistance(c);
    c.relPosition = cmatrix_.getWorldPositionByDirectDistance(c.centerX, c.centerY, directDistance);
    c.groundDistance = cmatrix_.groundDistance(c.relPosition);
    c.relPosition.z -= BALL_RADIUS;
    c.valid = true;
    auto& self = getself();
    Point2D cand = Point2D(c.relPosition.x, c.relPosition.y).relativeToGlobal(self.loc, self.orientation);
    c.absPosition = Position(cand.x, cand.y, 0);
    // get ball velocity
    float dist = (ball.loc - cand).getMagnitude();
    int frames = getframe() - ball.frameLastSeen;
    if (frames > 0)
        c.velocity = dist / frames;
    else
        c.velocity = 0;
#ifdef TOOL
    c.velocity = 0; // Velocity can be problematic when running logs, especially when log frames aren't sequential
#endif
    // find best ball estimate
    if(conf > curBestScore) {
        curBestScore = conf;
        bestCandId = i;
    }
    candidates_.push_back(c);
    // confidence.push_back(conf);
    // vector<float> box = {
    //   interpreter->typed_output_tensor<float>(0)[(5 + num_classes) * i + 0],
    //   interpreter->typed_output_tensor<float>(0)[(5 + num_classes) * i + 1],
    //   interpreter->typed_output_tensor<float>(0)[(5 + num_classes) * i + 2],
    //   interpreter->typed_output_tensor<float>(0)[(5 + num_classes) * i + 3]
    // };
    // boxes.push_back(box);
  }
  // set best ball
  if (bestCandId != -1) {
    setBest(candidates_[bestCandidateIndex]);
  }
*/
  
  // std::cerr << "Frame completed YOLO" << endl;
  counter_++;
  VisionTimer::Stop("BallDetector(%s)::findBall", camera_);
  return;
}


// Ishan: assuming we won't need this
bool YoloBallDetector::findMovingBall() {
  return false;

}



/**
 * Attempts to fit a perfect circle on the points of the given ball candidate.
 * might not need this method. Keeping just in case - Ishan
 */
void YoloBallDetector::fitCircleToPoints(
                                     uint16_t *x, uint16_t *y, uint16_t n,               // in
                                     float *cx, float *cy, float *radius, float *stddev) // out
{
  *cx = 0; *cy = 0; *radius = 0; *stddev = 1e6;
  if (n < 3) return; // ERROR: fewer than three points

  // computing centroids
  float centroidX = 0, centroidY = 0;
  for (uint16_t i = 0; i < n; i++) {
    centroidX += x[i]; centroidY += y[i]; }
  centroidX /= n; centroidY /= n;

  // computing moments
  float m_xx = 0, m_xy = 0, m_xz = 0, m_yy = 0, m_yz = 0, m_zz = 0;
  for (uint16_t i = 0; i < n; i++) {
    float xi = x[i] - centroidX, yi = y[i] - centroidY;
    float zi = xi * xi + yi * yi;
    m_xx += xi * xi; m_xy += xi * yi; m_xz += xi * zi;
    m_yy += yi * yi; m_yz += yi * zi; m_zz += zi * zi;
  }
  m_xx /= n; m_xy /= n; m_xz /= n; m_yy /= n; m_yz /= n; m_zz /= n;

  // computing the coefficients of the characteristic polynomial
  float m_z = m_xx + m_yy;
  float cov_xy = m_xx * m_yy - m_xy * m_xy;
  float a3 = 4 * m_z;
  float a2 = -3 * m_z * m_z - m_zz;
  float a1 = m_zz * m_z + 4 * cov_xy * m_z;
  a1 -= m_xz * m_xz + m_yz * m_yz + m_z * m_z * m_z;
  float a0 = m_xz * m_xz * m_yy + m_yz * m_yz * m_xx;
  a0 -= m_zz * cov_xy + 2 * m_xz * m_yz * m_xy - m_z * m_z * cov_xy;
  float a22 = a2 + a2;
  float a33 = a3 + a3 + a3;
  float newX = 0;
  float newY = 1e20;
  const float eps = 1e-12;
  const size_t MAX_ITER = 20;

  // newton's method starting at x = 0
  for (size_t i = 0; i < MAX_ITER; i++) {
    float oldY = newY;
    newY = a0 + newX * (a1 + newX * (a2 + newX * a3));
    if (fabs(newY) > fabs(oldY))
      return; // ERROR: newton-taubin going wrong dir.
    float dy = a1 + newX * (a22 + newX * a33);
    float oldX = newX;
    newX = oldX - newY / dy;
    if (fabs((newX - oldX) / newX) < eps)
      break; // converged!
    if (newX < 0)
      newX = 0; // ERROR: newton-taubin having neg. root
  }

  // computing the circle parameters
  float det = 2 * (newX * newX - newX * m_z + cov_xy);
  if (fabs(det) < 1e-3) return; // ERROR: zero determinant

  *cx = (m_xz * (m_yy - newX) - m_yz * m_xy) / det;
  *cy = (m_yz * (m_xx - newX) - m_xz * m_xy) / det;
  *radius = sqrtf(*cx * *cx + *cy * *cy + m_z);
  *cx += centroidX; *cy += centroidY;

  // compute standard deviation with a normalized circle (r = 1)
  float m = 0, v = 0;
  for (uint16_t i = 0; i < n; i++) {
    float dx = x[i] - *cx, dy = y[i] - *cy;
    float d = sqrtf(dx * dx + dy * dy) / *radius;
    v += d * d; m += d;
  }
  m /= n; v /= n;
  *stddev = sqrtf(v - m * m);
}



float YoloBallDetector::max(float a, float b, float c) {
  return std::max(std::max(a, b), c);
}

float YoloBallDetector::min(float a, float b, float c) {
  return std::min(std::min(a, b), c);
}


// ScanLine* YoloBallDetector::getRoot(ScanLine& sl) {
//   ScanLine* root = &sl;
//   while(root->parent != NULL) {
//     root = root->parent;
//   }
//   return root;
// }
// 
// void YoloBallDetector::merge(ScanLine& top, ScanLine& bot) {
//   ScanLine* topRoot = getRoot(top);
//   ScanLine* botRoot = getRoot(bot);
//   if(bot.parent == NULL) {
//     topRoot->children.push_back(&bot);
//     bot.parent = topRoot;
//   } else if (topRoot != botRoot) {
//     botRoot->children.insert(botRoot->children.end(), topRoot->children.begin(), topRoot->children.end());
//     topRoot->parent = botRoot;
//   }
// }




// void YoloBallDetector::refineBallLocation(BallCandidate& candidate, const ROI* roi, cv::Mat& mat, BlobTriangle triangle) {
//   // erase black spots on ball
//   eraseBlob(roi->mat, triangle.b1);
//   eraseBlob(roi->mat, triangle.b2);
//   eraseBlob(roi->mat, triangle.b3);
// 
//   // only run this function on the bottom camera
//   // xstep and ystep should be the same
//   std::vector<cv::Vec3f> circles;
//   int highCanny = 400;
//   double dp = 1;
//   double minDist = mat.rows; // only one circle allowed
//   double minRadius = candidate.radius/(roi->xstep)*0.95;
//   double maxRadius = candidate.radius/(roi->ystep)*1.2;
//   double accumulator_threshold = 3;
// 
//   // Rouhan's close ball (smaller version)
//   double high_thresh = cv::threshold(mat, mat.clone(), 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
//   highCanny = high_thresh;
//   /////
// 
//   cv::HoughCircles(mat, circles, CV_HOUGH_GRADIENT, dp, minDist, highCanny, accumulator_threshold, minRadius, maxRadius);
// 
//   if(circles.size() == 1) {
// 
//     int circlePadding = 5;
// 
//     Point2D circlePoint(circles[0][0], circles[0][1]);
//     float circleRadius = circles[0][2] + circlePadding;   // Add some padding in case blobs are on edge
//     int blobContainmentCount = 0;
//     if (Point2D(triangle.b1.centroidX, triangle.b1.centroidY).getDistanceTo(circlePoint) < circleRadius) {
//       blobContainmentCount++;
//     }
//     if (Point2D(triangle.b2.centroidX, triangle.b2.centroidY).getDistanceTo(circlePoint) < circleRadius) {
//       blobContainmentCount++;
//     }
//     if (Point2D(triangle.b3.centroidX, triangle.b3.centroidY).getDistanceTo(circlePoint) < circleRadius) {
//       blobContainmentCount++;
//     }
// 
//     if (blobContainmentCount >= 2) {
//       // if the hough circle contains fewer than 2 blobs, don't use it
//       candidate.centerX = circles[0][0]*roi->xstep + roi->xmin;
//       candidate.centerY = circles[0][1]*roi->ystep + roi->ymin;
//       candidate.radius = circles[0][2]*roi->xstep;  // xstep should equal ystep
//     }
//   }
// }


void YoloBallDetector::selectRobots(std::vector<RobotCandidate>& candidates) {
    std::sort(candidates.begin(), candidates.end(), RobotCandidate::sortPredicate);
    const unsigned int count = WO_OPPONENT_LAST - WO_OPPONENT_FIRST + 1;
    int oppOffset = 0;
    for(unsigned int i = 0; i < std::min(count, (const unsigned int)candidates.size()); i++) {
        RobotCandidate& candidate = candidates[i];
        if(candidate.confidence < .3) break;
        WorldObject *wo = &vblocks_.world_object->objects_[WO_OPPONENT_FIRST + i + oppOffset];
        while(wo->seen && i + oppOffset < count) {
            oppOffset++;
            wo = &vblocks_.world_object->objects_[WO_OPPONENT_FIRST + i + oppOffset];
        }
        if(i + oppOffset >= count) break;

        wo->visionDistance = cmatrix_.groundDistance(candidate.relPosition);
        wo->visionBearing = cmatrix_.bearing(candidate.relPosition);
        wo->visionElevation = cmatrix_.elevation(candidate.relPosition);
        wo->imageCenterX = candidate.avgX;
        wo->imageCenterY = candidate.yi;
        wo->fromTopCamera = (camera_ == Camera::TOP);
        wo->seen = true;

    }
}


bool YoloBallDetector::setBest(BallCandidate& candidate) {
  candidate.best = true;
  best_ = std::make_unique<BallCandidate>(candidate);
  auto& ball = getball();
  ball.reset();
  ball.radius = candidate.radius;
  ball.imageCenterX = candidate.centerX;
  ball.imageCenterY = candidate.centerY;
  ball.visionDistance = candidate.groundDistance;
  ball.visionBearing = cmatrix_.bearing(candidate.relPosition);
  ball.visionElevation = cmatrix_.elevation(candidate.relPosition);
  ball.seen = true;
  ball.visionConfidence = candidate.confidence;
  ball.frameLastSeen = getframe();
  ball.fromTopCamera = (camera_ == Camera::TOP);
  #ifndef TOOL
  tlog(34,"selected ball at %3.0f,%3.0f dist=%2.2f, bear=%2.2f, elev=%2.2f",
    candidate.absPosition.x, candidate.absPosition.y,
    ball.visionDistance, ball.visionBearing * RAD_T_DEG, ball.visionElevation * RAD_T_DEG
  );
  #endif

  return true;
}


bool YoloBallDetector::setCrossBest(CrossCandidate& candidate) {
  
  candidate.best = true;
  best_ = std::make_unique<CrossCandidate>(candidate);
  auto& ucross = getcross();
  ucross.reset();
  ucross.radius = candidate.radius;
  ucross.imageCenterX = candidate.centerX;
  ucross.imageCenterY = candidate.centerY;
  ucross.visionDistance = candidate.groundDistance;
  ucross.visionBearing = cmatrix_.bearing(candidate.relPosition);
  ucross.visionElevation = cmatrix_.elevation(candidate.relPosition);
  ucross.seen = true;
  ucross.visionConfidence = candidate.confidence;
  ucross.frameLastSeen = getframe();
  ucross.fromTopCamera = (camera_ == Camera::TOP);

  #ifndef TOOL
  tlog(34,"YOLO selected cross at %3.0f,%3.0f dist=%2.2f",
    candidate.absPosition.x, candidate.absPosition.y, ucross.visionDistance
  );
  #endif
  return true;
}

float YoloBallDetector::getDirectDistanceByBlobWidth(float dx, int /*imageX*/, int /*imageY*/) {
//  dx *= 640.0f / iparams_.width; // Normalize because this was tuned with a width of 640
  //float dist = 24269.169211 * powf(dx,-0.904299);  // tuned for exploreUT13
//  float dist = 27615.38886 * powf(dx,-0.9571350451); // tuned for US Open 14 by katie
  float dist = cmatrix_.getWorldDistanceByWidth(dx, BALL_RADIUS*2);
  return dist;
}

float YoloBallDetector::getDirectDistanceByKinematics(int x, int y) {
  Position p = cmatrix_.getWorldPosition(x, y, BALL_RADIUS);
  float dist = cmatrix_.directDistance(p);
  return dist;
}

float YoloBallDetector::getDirectDistance(BallCandidate& candidate) {
  float dist;
  float wdist = getDirectDistanceByBlobWidth(2.0f*candidate.radius, candidate.centerX, candidate.centerY);
  float kdist = getDirectDistanceByKinematics(candidate.centerX, candidate.centerY);

  dist = kdist; // Less susceptible to errors from bounding boxes

  // // Kdist is better up close, wdist is better farther away. We scale the ratio of each so that we
  // // don't get large discrepancies at the cutoff points
  // if (kdist < 1200)
  //     dist = kdist;
  // else
  //     dist = 0.45 * kdist + 0.55 * wdist;
  return dist;
}
