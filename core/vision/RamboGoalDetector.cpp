#include <vision/RamboGoalDetector.h>
#include <VisionCore.h>
#include <common/tinyformat.h>
#include <vision/ml/SvmClassifier.h>
#include <vision/BandSampler.h>
#include <vision/ColorSegmenter.h>
// #include <vision/ml/DeepClassifier.h>
#include <yuview/YUVImage.h>

std::string IMAGE_DIRECTORY = util::env("HOME") + "/images";

RamboGoalDetector::RamboGoalDetector(DETECTOR_DECLARE_ARGS, const ROIDetector& roi_detector, const ColorSegmenter& segmenter) : DETECTOR_INITIALIZE, roi_detector_(roi_detector), color_segmenter_(segmenter) {
}

RamboGoalDetector::~RamboGoalDetector() {
}

void RamboGoalDetector::init(TextLogger* tl) {
//   tfm::printf("Initializing %s goal detector.\n", camera_);
//   ObjectDetector::init(tl);
//   if(camera_ == Camera::BOTTOM) return;
//   auto path = VisionCore::inst_->memory_->data_path_;
//   classifier_ = std::make_unique<DeepClassifier>("goal", path + "/models");
//   tfm::printf("Loading deep network files for %s...", camera_); fflush(stdout);
//   classifier_->load();
//   tfm::printf("done!\n");
// #if defined(USER_jake) and defined(TOOL)
//   util::rmrf(IMAGE_DIRECTORY);
// #endif
//   util::mkdir_recursive(IMAGE_DIRECTORY);
}

void RamboGoalDetector::processFrame() {
//   ObjectDetector::processFrame();
//   if(processed_frames_ % 3 != 0) return;
//   if(camera_ == Camera::BOTTOM) return;
//   tlog(55, "Processing goal detection frame");
//   if(sampler_ != nullptr) {
//     tlog(55, "decaying weights");
//     sampler_->decayWeights();
//   }
//   VisionTimer::Start(55, "RGD::processFrame");
//   VisionTimer::Start(55, "RGD::Band sampling");
// 
//   // TODO: make this work for our own goal too
//   VisionTimer::Start(55, "RGD::Basic Data");
//   auto goal = vblocks_.world_object->objects_[WO_OPP_GOAL].loc;
//   auto self = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF].loc;
//   auto orient = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF].orientation;
//   auto distance = (goal - self).getMagnitude();
//   if(fabs(self.x) > HALF_FIELD_X - 1000 || distance < 1500) {
//     VisionTimer::Stop("RGD::processFrame");
//     VisionTimer::Stop("RGD::Band sampling");
//     VisionTimer::Stop("RGD::Basic Data");
//     return;
//   }
//   int expectedWidth = cmatrix_.getCameraWidthByDistance(distance, GOAL_POST_WIDTH) * 1.25;
//   int expectedHeight = std::max(8.0f, cmatrix_.getCameraHeightByDistance(distance, GOAL_POST_WIDTH)) * 1.25;
//   tlog(55, "Expected goal distance: %2.f, Exp pixel width: %i, height: %i", distance, expectedWidth, expectedHeight);
//   // If the band sampler isn't initialized or if the expected goal width is sufficiently different
//   float difference, average, threshold = 0.2f;
//   VisionTimer::Stop("RGD::Basic Data");
//   if(sampler_ != nullptr) {
//     difference = std::abs(static_cast<int>(sampler_->windowX()) - expectedWidth);
//     average = (sampler_->windowX() + expectedWidth) / 2.0f;
//     tlog(55, "Current sampler: %s\ndifference: %2.2f, average: %2.2f, threshold: %2.2f -> %2.2f", *sampler_, difference, average, threshold, threshold * average);
//   }
//   if(sampler_ == nullptr || difference > threshold * average) {
//     // Create a new sampler based on the expected height and width of the goal in image space
//     goal = goal.globalToRelative(self, orient);
//     auto expectedPosition = cmatrix_.getImageCoordinates(goal.x, goal.y, GOAL_HEIGHT + GOAL_POST_WIDTH / 2);
//     int ymax = 500; //TODO: compute this and recreate the sampler as necessary
//     tlog(55, "Creating a sampler with height: %i, width: %i, expected height: %i, expected width: %i",
//       ymax, iparams_.width, expectedHeight, expectedWidth
//     );
//     VisionTimer::Start(55, "RGD::createSampler");
//     auto sampler = std::make_unique<BandSampler>(ymax, iparams_.width, expectedHeight, expectedWidth);
//     sampler->init(textlogger);
//     VisionTimer::Stop(55, "RGD::createSampler");
//     if(sampler_ != nullptr) {
//       tlog(55, "Creating a new sampler with a prior spreading from %s", *sampler);
//       VisionTimer::Start(55, "RGD::setPrior");
//       sampler->setPrior(*sampler_);
//       VisionTimer::Stop("RGD::setPrior");
//     }
//     sampler_ = std::move(sampler);
//   }
//   auto posts = std::vector<Point2D> { 
//     vblocks_.world_object->objects_[WO_OPP_LEFT_GOALPOST].loc,
//     vblocks_.world_object->objects_[WO_OPP_RIGHT_GOALPOST].loc
//   };
//   for(auto post : posts) {
//     post = post.globalToRelative(self, orient);
//     auto expectedPosition = cmatrix_.getImageCoordinates(post.x, post.y, GOAL_HEIGHT + GOAL_POST_WIDTH / 2);
//     tlog(55, "Registering prior post at world position %2.2f, %2.2f, %2.2f back projected to image %i,%i",
//       post.x, post.y, GOAL_HEIGHT + GOAL_POST_WIDTH / 2, expectedPosition.x, expectedPosition.y
//     );
//     VisionTimer::Start(55, "RGD::registerPixel");
//     sampler_->registerPixel(expectedPosition.y, expectedPosition.x);
//     VisionTimer::Stop(55, "RGD::registerPixel");
//   }
//   int count = 4;
//   VisionTimer::Start(55, "RGD::sample");
//   auto bands = sampler_->sample(count);
//   VisionTimer::Stop("RGD::sample");
//   tlog(55, "sampling %i bands", count);
//   int bandnum = 0;
//   VisionTimer::Stop("RGD::Band sampling");
//   for(const Band& band : bands) {
//     VisionTimer::Start(55, "RGD::Band extraction");
//     cv::Rect rect(band.x, band.y, band.width, band.height);
//     auto roi = ROI(rect, camera_);
//     roi.xstep = roi.ystep = 1; // Always high-res for now
//     color_segmenter_.extractMat(roi, ColorSegmenter::Source::Raw);
//     //TODO: don't hard-code the source image size 
//     cv::resize(roi.mat, roi.mat, cv::Size(32, 32));
//     VisionTimer::Stop("RGD::Band extraction");
//     VisionTimer::Start(55, "RGD::DeepClassifier");
//     auto p = classifier_->classify(roi.mat);
//     VisionTimer::Stop("RGD::DeepClassifier");
// #if defined(USER_jake) and defined(TOOL)
//     auto m = yuview::YUVImage::FromMat(roi.mat).toMat(yuview::ImageFormat::RGB);
//     cv::imwrite(util::format("%s/frame_%i_band_%i_label_%i.png",
//       IMAGE_DIRECTORY, processed_frames_, ++bandnum, p), m
//     );
// #endif
//     if(p != Classifier::Nothing) {
//       tlog(55, "Matched band %s: %i", band, p);
//       sampler_->registerBand(band);
//       setGoalObject(band, p == 1); //TODO: Create enums for ML labels
//       break;
//     }
//   }
//   VisionTimer::Stop("RGD::processFrame");
}

void RamboGoalDetector::setGoalObject(const Band& band, bool left) {
//   // Increase the distance a bit since the ROI will be bigger than the post
//   float distance = cmatrix_.getWorldDistanceByWidth(band.width, GOAL_POST_WIDTH) * 1.2; 
//   auto position = cmatrix_.getWorldPositionByGroundDistance(band.centerX(), band.centerY(), distance);
//   float bearing = cmatrix_.bearing(position);
//   int index = left ? WO_UNKNOWN_LEFT_GOALPOST : WO_UNKNOWN_RIGHT_GOALPOST;
//   vblocks_.world_object->objects_[index].seen = true;
//   vblocks_.world_object->objects_[index].visionBearing = bearing;
//   vblocks_.world_object->objects_[index].visionDistance = distance;
//   vblocks_.world_object->objects_[index].imageCenterX = band.centerX();
//   vblocks_.world_object->objects_[index].imageCenterY = band.centerY();
//   vblocks_.world_object->objects_[index].frameLastSeen = vblocks_.frame_info->frame_id;
//   vblocks_.world_object->objects_[index].visionConfidence = 1.0; //TODO: use classifier certainty
//   vblocks_.world_object->objects_[index].fromTopCamera = (camera_ == Camera::TOP);
}
