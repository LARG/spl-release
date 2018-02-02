#include <vision/ImageProcessor.h>
#include <vision/LineDetector.h>
#include <vision/HoughDetector.h>
#include <vision/BallDetector.h>
#include <vision/BandRobotDetector.h>
#include <vision/JerseyRobotDetector.h>
#include <vision/HoughRobotDetector.h>
#include <vision/FieldEdgeDetector.h>
#include <vision/CrossDetector.h>
#include <vision/ColorSegmenter.h>
#include <vision/RamboGoalDetector.h>
#include <common/RobotCalibration.h>
#include <iostream>

#define DEBUG_TIMING false

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera)
{
  enableCalibration_ = false;
  color_segmenter_ = std::make_unique<ColorSegmenter>(vblocks_, vparams_, iparams_, camera_);
  roi_detector_ = std::make_unique<ROIDetector>(DETECTOR_PASS_ARGS, *color_segmenter_);
  hough_detector_ = std::make_unique<HoughDetector>(DETECTOR_PASS_ARGS, *color_segmenter_);
  blob_detector_ = std::make_unique<BlobDetector>(DETECTOR_PASS_ARGS, *color_segmenter_);
  line_detector_ = std::make_unique<LineDetector>(DETECTOR_PASS_ARGS, *color_segmenter_, *blob_detector_);
  rambo_goal_detector_ = std::make_unique<RamboGoalDetector>(DETECTOR_PASS_ARGS, *roi_detector_, *color_segmenter_);
  edge_goal_detector_ = std::make_unique<GoalDetector>(DETECTOR_PASS_ARGS, *color_segmenter_, *blob_detector_, *line_detector_, *hough_detector_); 
  ball_detector_ = std::make_unique<BallDetector>(DETECTOR_PASS_ARGS, *roi_detector_, *color_segmenter_, *blob_detector_);
  field_edge_detector_ = std::make_unique<FieldEdgeDetector>(DETECTOR_PASS_ARGS, *color_segmenter_, *blob_detector_);
  robot_detector_ = std::make_unique<RobotDetector>(DETECTOR_PASS_ARGS, *color_segmenter_, *blob_detector_, *hough_detector_);
  cross_detector_ = std::make_unique<CrossDetector>(DETECTOR_PASS_ARGS, *color_segmenter_, *blob_detector_);

  calibration_ = std::make_unique<RobotCalibration>();
}

ImageProcessor::~ImageProcessor() {
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  color_segmenter_->init(tl);
  roi_detector_->init(tl);
  blob_detector_->init(tl);
  line_detector_->init(tl);
  rambo_goal_detector_->init(tl);
  edge_goal_detector_->init(tl);
  ball_detector_->init(tl);
  robot_detector_->init(tl);
  cross_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

unsigned char* ImageProcessor::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable(){
  return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix(){
  return cmatrix_;
}

void ImageProcessor::updateTransform(){
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;

  Pose3D pcamera;
  if(enableCalibration_) {
    float joints[NUM_JOINTS], sensors[NUM_SENSORS], dimensions[RobotDimensions::NUM_DIMENSIONS];
    memcpy(joints, vblocks_.joint->values_.data(), NUM_JOINTS * sizeof(float));
    memcpy(sensors, vblocks_.sensor->values_.data(), NUM_SENSORS * sizeof(float));
    memcpy(dimensions, vblocks_.robot_info->dimensions_.values_, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
    Pose3D *rel_parts = vblocks_.body_model->rel_parts_.data(), *abs_parts = vblocks_.body_model->abs_parts_.data();
    calibration_->applyJoints(joints);
    calibration_->applySensors(sensors);
    calibration_->applyDimensions(dimensions);
    ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
    Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
    ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
    ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
    cmatrix_.setCalibration(*calibration_);
    pcamera = abs_parts[camera];
  }
  else pcamera = vblocks_.body_model->abs_parts_[camera];

  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH) {
    auto self = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF];
    pcamera.translation.z += self.height;
  }

  cmatrix_.updateCameraPose(pcamera);
}

bool ImageProcessor::isRawImageLoaded() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->isLoaded();
  return vblocks_.image->isLoaded();
}

int ImageProcessor::getImageHeight() {
  return iparams_.height;
}

int ImageProcessor::getImageWidth() {
  return iparams_.width;
}

double ImageProcessor::getCurrentTime() {
  return vblocks_.frame_info->seconds_since_start;
}

void ImageProcessor::setCalibration(const RobotCalibration& calibration){
  *calibration_ = calibration;
}

void ImageProcessor::processFrame(){
  VisionTimer::Start(30, "ImageProcessor(%s)::frame", camera_);
 
  VisionTimer::Start(30, "ImageProcessor(%s)::transforms", camera_);
  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM) return;
  tlog(30, "Process Frame camera %i", camera_);

  // Horizon calculation
  tlog(30, "Calculating horizon line");
  updateTransform();
  VisionTimer::Stop("ImageProcessor(%s)::transforms", camera_);
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 20000);
  vblocks_.robot_vision->horizon = horizon;

  tlog(30, "Classifying Image");
  color_segmenter_->setHorizon(horizon);
  
  VisionTimer::Start(30, "ImageProcessor(%s)::classification", camera_);
  if(!color_segmenter_->classifyImage(color_table_)) {
    VisionTimer::Stop("ImageProcessor(%s)::classification", camera_);
    VisionTimer::Stop("ImageProcessor(%s)::frame", camera_);
    return;
  }
  VisionTimer::Stop("ImageProcessor(%s)::classification", camera_);
  VisionTimer::Start(30, "ImageProcessor(%s)::runs", camera_);
  color_segmenter_->constructRuns();
  VisionTimer::Stop("ImageProcessor(%s)::runs", camera_);
  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH) {
    tlog(30, "COACH image: checking for balls in top camera");
    ball_detector_->setHorizon(horizon);
    auto rois = roi_detector_->findBallROIs();
    ball_detector_->findBall(rois);
    VisionTimer::Stop("ImageProcessor(%s)::frame", camera_);
    return;
  }
  tlog(30, "Preprocessing line points and forming blobs");
  VisionTimer::Start(30, "ImageProcessor(%s)::preProcess", camera_);
  color_segmenter_->preProcessPoints();
  VisionTimer::Stop("ImageProcessor(%s)::preProcess", camera_);

  tlog(30, "Detecting balls");
  ball_detector_->setHorizon(horizon);
  roi_detector_->setHorizon(horizon);
  VisionTimer::Start(30, "ImageProcessor(%s)::ball_rois", camera_);
  auto rois = roi_detector_->findBallROIs();
  VisionTimer::Stop("ImageProcessor(%s)::ball_rois", camera_);
  VisionTimer::Start(30, "ImageProcessor(%s)::balls", camera_);
  ball_detector_->findBall(rois);
  VisionTimer::Stop("ImageProcessor(%s)::balls", camera_);

  VisionTimer::Start(30, "ImageProcessor(%s)::blobs", camera_);
  blob_detector_->formWhiteLineBlobs();
  VisionTimer::Stop("ImageProcessor(%s)::blobs", camera_);

  VisionTimer::Start(30, "ImageProcessor(%s)::lines", camera_);
  tlog(30, "Detecting White Lines");
  
  vblocks_.robot_vision->lookForCross = true; // Always enabled for now
  if(camera_ == Camera::TOP && vblocks_.robot_vision->lookForCross) {
    tlog(30, "Detecting crosses");
    cross_detector_->detectCrosses();       // SN: this should happen after ball detection, so we don't find crosses on balls (until someone updates the cross detector)
  }
  else tlog(30, "Skipping cross detection");

  line_detector_->detectLines(horizon);
  VisionTimer::Stop("ImageProcessor(%s)::lines", camera_);

  field_edge_detector_->detectFieldEdges();

  tlog(30, "Detecting goals");
  VisionTimer::Start(30, "ImageProcessor(%s)::goals", camera_);
  hough_detector_->setHorizon(horizon);
  //rambo_goal_detector_->processFrame();
#ifndef TOOL
  if (vblocks_.frame_info->frame_id % 3 == 0){
#endif
  edge_goal_detector_->detectGoalPosts();
#ifndef TOOL
  }
#endif
  VisionTimer::Stop("ImageProcessor(%s)::goals", camera_);

  tlog(21, "Vision frame process complete");
  VisionTimer::Stop("ImageProcessor(%s)::frame", camera_);
}


int ImageProcessor::getTeamColor() {
  return vblocks_.robot_state->team_;
}

void ImageProcessor::SetColorTable(unsigned char* table) {
  color_table_ = table;
}

float ImageProcessor::getHeadChange() const {
  if (vblocks_.joint == NULL)
    return 0;
  return vblocks_.joint->getJointDelta(HeadPan);
}

std::vector<BallCandidate> ImageProcessor::getBallCandidates() {
  return ball_detector_->candidates();
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
  return ball_detector_->best();
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->isLoaded();
}
