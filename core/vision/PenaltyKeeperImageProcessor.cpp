#include "PenaltyKeeperImageProcessor.h"

#define DEBUG_DRAW false
#define DEBUG_ROBOT false
#define DEBUG false

PenaltyKeeperImageProcessor::PenaltyKeeperImageProcessor(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter) :
  DETECTOR_INITIALIZE, color_segmenter_(segmenter) {
    wholeRectLarge_ = cv::Rect(wholeXmin_, wholeYmin_, wholeRectWidth_, wholeRectHeight_);
    alignRect(wholeRectLarge_);
    state_ = IS_MOVING;
    prevBallMatSmall_ = cv::Mat(0, 0, CV_8UC1);
    ballRectLarge_ = cv::Rect(0, 0, 0, 0);
    if (DEBUG) std::cout << "PenaltyKeeperImageProcessor::constructor state_ = " << state_ << std::endl;
}

//////////////////////////////////////////////////////////////

cv::Rect PenaltyKeeperImageProcessor::large2small(cv::Rect& r) {
  return cv::Rect(
    (r.x - wholeXmin_)/xstep_,
    (r.y - wholeYmin_)/ystep_,
    r.width/xstep_,
    r.height/ystep_);
}

void PenaltyKeeperImageProcessor::alignRect(cv::Rect& rect, int xstep, int ystep) {
  // align rect so that resizing the ColorSegmenter's image will hit the right pixels
  // rect scale is original image resolution (1280 x 960)
  rect.x -= rect.x % xstep;
  rect.y -= rect.y % ystep;
  rect.width += (xstep - (rect.width % xstep));
  rect.height += (ystep - (rect.height % ystep));
}

//////////////////////////////////////////////////////////////

cv::Point PenaltyKeeperImageProcessor::findBottomPoint(cv::Mat& mat) {
  int ymin = -1;
  int xavg = 0;
  int xcount = 0;
  for(int i = mat.rows - 1; i >= ymin - 6; --i) {
      for(int j = 0; j < mat.cols; ++j) {
          if (mat.at<uchar>(i, j) == 255) {
            if (ymin == -1) {
              ymin = i;
            }
            xavg += j;
            xcount++;
          }
      }
  }
  if(xcount == 0) {
    return cv::Point(-1,-1);
  }

  xavg /= xcount;
  
  cv::Point center(xavg, ymin);

  return center;
}

bool PenaltyKeeperImageProcessor::enoughBallEvidence(cv::Rect& avgRect) {
  int minNumRects = 5;
  int n = ballRectHistory_.size();
  if (n < minNumRects) {
    return false;
  }

  // compute "intersection over union" of temporally adjacent rectangles
  cv::Rect r1, r2;
  int xAvg = 0, yAvg = 0, widthAvg = 0, heightAvg = 0;
  double intersectionArea, unionArea;
  for(int i = n-1; i > n-minNumRects; i--) {
    r1 = ballRectHistory_[i];
    r2 = ballRectHistory_[i-1];
    intersectionArea = (r1 & r2).area();
    unionArea = r1.area() + r2.area() - intersectionArea;
    double iou = intersectionArea/unionArea;
    
    if (iou < 0.75) {
      return false;
    }
    
    xAvg += r1.x;
    yAvg += r1.y;
    widthAvg += r1.width;
    heightAvg += r1.height;
  }
  
  xAvg /= (minNumRects-1);
  yAvg /= (minNumRects-1);
  widthAvg /= (minNumRects-1);
  heightAvg /= (minNumRects-1);

  avgRect = cv::Rect(xAvg, yAvg, widthAvg, heightAvg);
  std::cout << "\tenoughBallEvidence: true! avgRect = " << avgRect << std::endl;
  return true;
}

void PenaltyKeeperImageProcessor::findBall() {
  cv::Mat wholeMatSmall = color_segmenter_.extractGrayscaleMat(wholeRectLarge_, xstep_, ystep_);

  if(DEBUG_DRAW) {
    char buff[100];
    snprintf(buff, sizeof(buff), "/home/%s/log_images/penaltyKeeper/%05d_0_wholeMatSmall.png", user_, topFrameCounter_);
    std::string filepath = buff;
    bool success = cv::imwrite(filepath, wholeMatSmall);  
    if (!success) {
      std::cout << "imwrite failed: " << filepath << std::endl;
    }
  }

  cv::Mat binaryFrame;
  cv::adaptiveThreshold(wholeMatSmall, binaryFrame, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, window_, threshold_);
  cv::Point bottomPoint = findBottomPoint(binaryFrame);

  if(DEBUG_DRAW) {
    if(bottomPoint.x != -1) {
      int radius = 2;
      cv::circle(binaryFrame, bottomPoint, radius, cv::Scalar(200,200,200), 1, 8, 0);
    }

    char buff[100];
    snprintf(buff, sizeof(buff), "/home/%s/log_images/penaltyKeeper/%05d_1_binaryFrame.png", user_, topFrameCounter_);
    std::string filepath = buff;
    bool success = cv::imwrite(filepath, binaryFrame);  
    if (!success) {
      std::cout << "imwrite failed: " << filepath << std::endl;
    }
  }

  if(bottomPoint.x == -1) {
    return;
  }

  int midX = bottomPoint.x*xstep_ + wholeXmin_;
  int bottomY = bottomPoint.y*ystep_ + wholeYmin_;
  float ballDiameter = cmatrix_.getExpectedCameraWidth(midX, bottomY, 0, 100.0);
  float radius = ballDiameter/2;
  radius *= 0.8;
  cv::Rect ballRectEvidence(midX - radius, bottomY - 2*radius, 2*radius, 2*radius);
  ballRectHistory_.push_back(ballRectEvidence);

  // draw estimated ball rect
  if(DEBUG_DRAW) {
    cv::Rect ballRectSmall = large2small(ballRectEvidence);
    cv::rectangle(binaryFrame, ballRectSmall, cv::Scalar(100, 100, 100));

    char buff[100];
    snprintf(buff, sizeof(buff), "/home/%s/log_images/penaltyKeeper/%05d_2_ballRectEvidence.png", user_, topFrameCounter_);
    std::string filepath = buff;
    bool success = cv::imwrite(filepath, binaryFrame);
    if (!success) {
      std::cout << "imwrite failed: " << filepath << std::endl;
    }
  }

  cv::Rect avgRect(1, 1, 1, 1);
  if(enoughBallEvidence(avgRect)) {
    if(avgRect.x == 1 && avgRect.width == 1) {
      std::cout << "ERROR: avgRect must not have been set correctly: avgRect = " << avgRect << std::endl;
    }

    // set ball region
    ballRectLarge_ = avgRect;
    alignRect(ballRectLarge_);

    // the region below the ball
    belowRectLarge_ = cv::Rect(
      avgRect.x + (avgRect.width/2) - 200,
      avgRect.y + avgRect.height + 20,
      400,
      200
    );
    alignRect(belowRectLarge_);

    state_ = WATCH_BALL_AND_FIND_ROBOT;
  }
}

//////////////////////////////////////////////////////////////

bool PenaltyKeeperImageProcessor::ballMoved() {
  bool movementDetected = false;

  cv::Mat curBallMatSmall = color_segmenter_.extractGrayscaleMat(ballRectLarge_, xstep_, ystep_);
  cv::Mat curBelowMatSmall = color_segmenter_.extractGrayscaleMat(belowRectLarge_, xstep_, ystep_);
  
  if(DEBUG_DRAW) {
    char buff[100];
    snprintf(buff, sizeof(buff), "/home/%s/log_images/penaltyKeeper/%05d_3_curBallMat.png", user_, topFrameCounter_);
    std::string filepath = buff;
    bool success = cv::imwrite(filepath, curBallMatSmall);
    if (!success) {
      std::cout << "imwrite failed: " << filepath << std::endl;
    }

    snprintf(buff, sizeof(buff), "/home/%s/log_images/penaltyKeeper/%05d_4_curBelowMat.png", user_, topFrameCounter_);
    filepath = buff;
    success = cv::imwrite(filepath, curBelowMatSmall);  
    if (!success) {
      std::cout << "imwrite failed: " << filepath << std::endl;
    }
  }

  if (prevBallMatSmall_.rows != 0) {
    // If prevBallMatSmall_.rows == 0 then this must be the first run through
    // the ballMoved() function, and We don't have a prev frame yet.

    // look for movement on ball
    cv::Mat diffBallMatSmall;
    cv::absdiff(curBallMatSmall, prevBallMatSmall_, diffBallMatSmall);
    float meanBallDiff = cv::mean(diffBallMatSmall)[0];
    
    cv::Mat diffBelowMatSmall;
    cv::absdiff(curBelowMatSmall, prevBelowMatSmall_, diffBelowMatSmall);
    float meanBelowDiff = cv::mean(diffBelowMatSmall)[0];

    float ratio = meanBallDiff/meanBelowDiff;
    movementDetected = meanBallDiff > DIFF_BALL_THRESHOLD && ratio > DIFF_RATIO_THRESHOLD;
    
    if (movementDetected) {
      std::cout << "ball movement detected!" << std::endl;
      std::cout << "\tmeanBallDiff = " << meanBallDiff << std::endl;
      std::cout << "\tmeanBelowDiff = " << meanBelowDiff << std::endl;
      std::cout << "\tratio = " << ratio << std::endl;
    }
  }
  
  prevBallMatSmall_ = curBallMatSmall;
  prevBelowMatSmall_ = curBelowMatSmall;

  return movementDetected;
}

//////////////////////////////////////////////////////////////

bool PenaltyKeeperImageProcessor::pointInRect(int y, int x, cv::Rect& r) {
  return r.y <= y && y <= r.y + r.height &&
         r.x <= x && x <= r.x + r.width;
}

void PenaltyKeeperImageProcessor::findRobot() {
  if (DEBUG) std::cout << "PenaltyKeeperImageProcessor::findRobot (enter)" << std::endl;

  int padding = 40;
  cv::Rect robotZoneLarge(
    ballRectLarge_.x - 3*padding,
    ballRectLarge_.y - 3*padding,
    ballRectLarge_.width + 6*padding,
    ballRectLarge_.height + 3*padding);
  alignRect(robotZoneLarge);

  cv::Mat frame = color_segmenter_.extractGrayscaleMat(robotZoneLarge, xstep_, ystep_);

  cv::Mat binaryFrame;
  int threshold = 10, window = 5;
  cv::adaptiveThreshold(frame, binaryFrame, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, window, threshold);

  int px = padding/xstep_;
  int py = padding/ystep_;
  cv::Rect ballRect(3*px, 3*py, ballRectLarge_.width/xstep_, ballRectLarge_.height/ystep_);

  cv::Rect deadZoneSmall = ballRect;
  deadZoneSmall.width += 2;
  deadZoneSmall.y -= 2;
  deadZoneSmall.height += 2;

  // int ymin = 10000;
  // int ymax = -1;
  // int xmin = 10000;
  // int xmax = -1;
  int yavg = 0;
  int xavg = 0;
  int count = 0;
  for(int i = 0; i < binaryFrame.rows; ++i) {
    for(int j = 0; j < binaryFrame.cols; ++j) {
      if (pointInRect(i, j, deadZoneSmall)) {
        continue;
      }
      if (binaryFrame.at<uchar>(i, j) == 255) {
        binaryFrame.at<uchar>(i, j) = 50;
        // ymin = std::min(ymin, i);
        // ymax = std::max(ymax, i);
        // xmin = std::min(xmin, j);
        // xmax = std::max(xmax, j);
        // std::cout << ymin << " " << ymax << " " << xmin << " " << xmax << std::endl;
        yavg += i;
        xavg += j;
        count++;
      }
    }
  }

  if (count == 0) {
  if (DEBUG) std::cout << "PenaltyKeeperImageProcessor::findRobot (return count = 0)" << std::endl;
    return;
  }

  xavg /= count;
  yavg /= count;
  
  int xmid = binaryFrame.cols/2;
  int offset = xavg - xmid;
  if (DEBUG) std::cout << "PenaltyKeeperImageProcessor::findRobot offset = " << offset << std::endl;

  auto& wo_robot = vblocks_.world_object->objects_[WO_OPPONENT1];
  wo_robot.imageCenterX = offset;

  if (DEBUG_ROBOT) {
    if (DEBUG) std::cout << "PenaltyKeeperImageProcessor::findRobot topFrameCounter_ = " << topFrameCounter_ << std::endl;

    cv::Point center(xavg, yavg);
    int radius = 1;

    cv::rectangle(frame, ballRect, cv::Scalar(50, 50, 50));
    cv::rectangle(frame, deadZoneSmall, cv::Scalar(100, 100, 100));
    cv::circle(frame, center, radius, cv::Scalar(255,255,255), 1, 8, 0);

    cv::rectangle(binaryFrame, ballRect, cv::Scalar(50, 50, 50));
    cv::rectangle(binaryFrame, deadZoneSmall, cv::Scalar(100, 100, 100));
    cv::circle(binaryFrame, center, radius, cv::Scalar(255,255,255), 1, 8, 0);

    char buff[100];
    snprintf(buff, sizeof(buff), "/home/%s/log_images/penaltyKeeper/%05d_robot_0_%02d_%02d.png", user_, topFrameCounter_, threshold, window);
    std::string filepath = buff;
    bool success = cv::imwrite(filepath, frame);  
    if (!success) {
      std::cout << "imwrite failed: " << filepath << std::endl;
    }

    snprintf(buff, sizeof(buff), "/home/%s/log_images/penaltyKeeper/%05d_robot_1_%02d_%02d.png", user_, topFrameCounter_, threshold, window);
    filepath = buff;
    success = cv::imwrite(filepath, binaryFrame);  
    if (!success) {
      std::cout << "imwrite failed: " << filepath << std::endl;
    }
  }
  if (DEBUG) std::cout << "PenaltyKeeperImageProcessor::findRobot (return)" << std::endl;
}

//////////////////////////////////////////////////////////////

void PenaltyKeeperImageProcessor::resetVariables() {
  /* reset a bunch of variables */

  // std::cout << "resetVariables (enter)" << std::endl;

  // mark the ball as not moved
  auto& wo_ball = vblocks_.world_object->objects_[WO_BALL];
  // std::cout << "resetVariables A" << std::endl;
  wo_ball.ballMoved = false;
  // std::cout << "resetVariables B" << std::endl;

  auto& wo_robot = vblocks_.world_object->objects_[WO_OPPONENT1];
  // std::cout << "resetVariables C" << std::endl;
  wo_robot.imageCenterX = -9999;
  // std::cout << "resetVariables D" << std::endl;

  vblocks_.behavior->penaltyKeeperWatchBall = false;
  // std::cout << "resetVariables E" << std::endl;
  
  prevBallMatSmall_ = cv::Mat(0, 0, CV_8UC1);
  // std::cout << "resetVariables F" << std::endl;

  ballRectLarge_ = cv::Rect(0, 0, 0, 0);
  // std::cout << "resetVariables G" << std::endl;

  // clear all ball detections
  ballRectHistory_.clear();
  // std::cout << "resetVariables H" << std::endl;

  state_ = IS_MOVING;
  // std::cout << "resetVariables (return)" << std::endl;
}

bool PenaltyKeeperImageProcessor::penaltyKeeperProcessFrame(){
  if (DEBUG) std::cout << "PenaltyKeeperImageProcessor::penaltyKeeperProcessFrame (enter)" << std::endl;
  if (camera_ == Camera::BOTTOM) {
    if (DEBUG) std::cout << "PenaltyKeeperImageProcessor::penaltyKeeperProcessFrame (return bottom)" << std::endl;
    return false;
  }

  topFrameCounter_++;

  // ballRectLarge_ = cv::Rect(609, 321, 100, 100);
  // ballRectLarge_ = cv::Rect(572, 370, 100, 100);
  // state_ = WATCH_BALL_AND_FIND_ROBOT;

  if(vblocks_.game_state->state() != PLAYING || !vblocks_.behavior->penaltyKeeperWatchBall) {
    resetVariables();
  }

  if (vblocks_.behavior->penaltyKeeperWatchBall && state_ == IS_MOVING) {
    state_ = FIND_BALL;
  }

  switch(state_) {
    case IS_MOVING: {
      // look for ball in the normal way
      if (DEBUG) std::cout << "PenaltyKeeperImageProcessor::penaltyKeeperProcessFrame (return is_moving)" << std::endl;
      return true;
    }

    case FIND_BALL: {
      findBall();
      break;
    }

    case WATCH_BALL_AND_FIND_ROBOT: {
      bool didMove = ballMoved();
      // check for ball movement first
      if (didMove) {
        auto& wo_ball = vblocks_.world_object->objects_[WO_BALL];
        wo_ball.reset();
        wo_ball.ballMoved = true;
        wo_ball.frameLastSeen = vblocks_.frame_info->frame_id;
        state_ = BALL_MOVEMENT_DETECTED;
        break;
      }

      if (ballRectLarge_.width > 0) {
        findRobot();
      }
      break;
    }

    case BALL_MOVEMENT_DETECTED:  {
      // don't need to do anything.
      break;
    }

    default:
      std::cout << "ERROR: shouldn't reach default switch case." << std::endl;
      break;
  }
  if (DEBUG) std::cout << "PenaltyKeeperImageProcessor::penaltyKeeperProcessFrame (return)" << std::endl;
  return false;
}