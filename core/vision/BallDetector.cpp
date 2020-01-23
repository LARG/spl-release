#include <vision/BallDetector.h>
#include <vision/ROIDetector.h>
#include <vision/ColorSegmenter.h>
#include <vision/ml/SvmClassifier.h>
// #include <vision/ml/DeepClassifier.h>
#include <VisionCore.h>


#define horizontalBlob blob_detector_.horizontalBlob
#define getball() vblocks_.world_object->objects_[WO_BALL]
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

BallDetector::BallDetector(DETECTOR_DECLARE_ARGS, const ROIDetector& roi_detector, ColorSegmenter& segmenter, BlobDetector& blob_detector, FieldEdgeDetector& field_edge_detector) : DETECTOR_INITIALIZE, roi_detector_(roi_detector), color_segmenter_(segmenter), blob_detector_(blob_detector), field_edge_detector_(field_edge_detector) {
  estimator_.setMean(
    1.0f, // green below percent
    0.0f, // ball green percent
    0.0f, // height
    0.0f, // kw disc
    0.0f, // field dist
    0.0f, // velocity
    0.5f  // triangle score -- a score of 0.5 is a perfect equilateral triangle. It represents cos(theta) where theta is the largest triangle angle.
  );

  // TODO: sanmit
  movingball_estimator_.setMean(
    0.70f, // ball white pixel %
    0.2f, // ball undef pixel %
    0.9f, // ball pixel %
    1.0f, // green below percent
    0.0f, // circle deviation
    0.0f, // height
    0.0f // kw disc
//    0.0f, // field dist
//    0.0f // velocity
  );

  penaltyball_estimator_.setMean(
    1.0f, // ball pixel %
    1.0f, // green below %
    0.0f, // circle deviation
    0.0f, // height
    0.0f, // kw disc
    0.0f // field dist
//    0.0f  // velocity
  );
}

BallDetector::~BallDetector() {
}

void BallDetector::init(TextLogger* tl) {
  printf("Initializing %s ball detector.\n", Camera::c_str(camera_));
  ObjectDetector::init(tl);
  auto path = VisionCore::inst_->memory_->data_path_ + "/models";
  printf("Loading ball detector files for %s...", Camera::c_str(camera_)); fflush(stdout);
  classifier_ = std::make_unique<SvmClassifier>("ball", path);
  classifier_->load();
  printf("done!\n");
}

void BallDetector::findBall(std::vector<ROI>& rois) {
  VisionTimer::Start(70, "BallDetector(%s)::findBall", camera_);
  if(camera_ == Camera::BOTTOM)
    vblocks_.roi->ball_rois_.clear();
 
  best_.reset();
  candidates_.clear();

#ifdef TOOL
  blobDebugImg = cv::Mat(iparams_.height, iparams_.width, CV_8UC3);
  blobDebugImg.setTo(cv::Scalar(255,255,255));
#endif

  WorldObject &ball = getball();
  // if ball was already detected by bottom camera, and we're in top cam, done
  if(ball.seen && !ball.fromTopCamera && camera_ == Camera::TOP) {
    tlog(70, "Ball was seen already in the bottom camera, bailing out.");
    VisionTimer::Stop("BallDetector(%s)::findBall", camera_);
    counter_++;     // So that debug_blob images don't overwrite between top and bottom cam
    return;
  }

  if(DEBUG_WRITE_ROIS && camera_ != Camera::TOP) {
    VisionTimer::Stop("BallDetector(%s)::findBall", camera_);
    return;
  }

  roiCounter_ = -1;  
  float curBestScore = 0;
  int bestCandidateIndex = -1;
  ROI* best = nullptr;
  cv::Mat best_mat;
  BlobTriangle best_triangle;
  
  float ballDistance = ball.relPos.getMagnitude();
  bool ballInitialized = ball.frameLastSeen > 0;
  float maxV = 100;
  float maxD = 100;
  // Scale max velocity with expected distance since positional uncertainty increases w/ range
  if(ballInitialized) {
    maxV = std::max(100.0f, ballDistance * .1f);
    maxD = std::max(100.0f, ballDistance * .2f);
  }
  estimator_.setStdDev(
    0.4f,   // mean 1.0 green below percent
    0.4f,   // mean 0.0 ball green percent
    250.0f, // mean 0.0 height
    0.4f,   // mean 0.0 kw disc
    maxD,   // mean 0.0 field dist
    maxV,    // mean 0.0 velocity
    0.175f    // triangle score deviation -- roughly 10 degrees in each std dev. so 60-70 is in first sigma. 
  );

  int default_hscale = 3;  // xstep = 8
  int default_vscale = 2;  // ystep = 4
  if(camera_ == Camera::BOTTOM) {
    default_hscale = 1;  // xstep = 2
    default_vscale = 1;   // ystep = 2
  }

  for(auto& roi : rois) {
    int xstep = roi.xstep;
    int ystep = roi.ystep;
   
    tlog(70, "-----------------------");
    tlog(70, "checking ROI: %s", roi);
    
    roiCounter_++;

    // Do high res scan if need be
//    VisionTimer::Start(70, "BallDetector(%s)::highres", camera_);
    bool highres = false;
    if(camera_ == Camera::TOP && (roi.hscale < default_hscale || roi.vscale < default_vscale)) {
      tlog(70, "Doing high res scan for ball");
      highres = true;
      color_segmenter_.setStepScale(roi.hscale, roi.vscale);
      const FocusArea focusArea(roi);
      color_segmenter_.classifyImage(focusArea);
      color_segmenter_.setStepScale(default_hscale, default_vscale);
    }
//    VisionTimer::Stop("BallDetector(%s)::highres", camera_);

//    VisionTimer::Start(70, "BallDetector(%s)::extractMat", camera_);
    cv::Mat mat = color_segmenter_.extractMat(roi);
//    VisionTimer::Stop("BallDetector(%s)::extractMat", camera_);
    // cv::Mat mat;
    // cv::resize(color_segmenter_.m_img, mat, cv::Size(), 1.0 / xstep, 1.0 / ystep, cv::INTER_NEAREST); 
    
    // apply heuristic tests with blobs
//    VisionTimer::Start(70, "BallDetector(%s)::blobTests", camera_);
    auto triangle = blobTests(mat, roi, xstep, ystep, highres);
    tlog(70, "triangle.score = %f", triangle.score);
//    VisionTimer::Stop("BallDetector(%s)::blobTests", camera_);

    if(triangle.score < 0) continue;

    BallCandidate c;
    c.centerX = triangle.centerX;
    c.centerY = triangle.centerY;
    c.radius = triangle.radius;
    c.width = roi.xmax - roi.xmin;
    c.height = roi.ymax - roi.ymin;
    float directDistance = getDirectDistance(c);
    c.relPosition = cmatrix_.getWorldPositionByDirectDistance(c.centerX, c.centerY, directDistance);
    c.groundDistance = cmatrix_.groundDistance(c.relPosition);
    c.relPosition.z -= BALL_RADIUS;
    c.valid = true;
    auto& self = getself();
    Point2D cand = Point2D(c.relPosition.x, c.relPosition.y).relativeToGlobal(self.loc, self.orientation);

    //tlog(70, "relposition (%f,%f) self (%f,%f) %f", c.relPosition.x, c.relPosition.y, self.loc.x, self.loc.y, self.orientation);

    c.absPosition = Position(cand.x, cand.y, 0);

    
    float belowGreenPct = camera_ == Camera::TOP ? checkBelowGreenPct(c, true) : 1.0;
    // Hard constraint on green percentage -- may want to make soft/probabilistic later.
    tlog(70, "below green pct: %2.2f", belowGreenPct);
    //if (belowGreenPct < 0.6) continue;

    // Ideally discrepancy should be 0, but on average, for good candidates it's usually around 0.3. After 0.4 it gets worrisome, and after 0.45 they are pretty much all bad. 
    tlog(70, "kw discrepancy: %2.2f", c.kwDistanceDiscrepancy);
    //if (c.kwDistanceDiscrepancy > 0.45) continue;

    // This is currently used just to rule out candidates on the net. We could change it to check for white/undefined percentage instead of green.  
    float ballGreenPct = checkBallColor(c); 

    // This is extremely loose because the color table is not that accurate, and the ball boundary isn't either. Sometimes helps guard against net detections.  
    tlog(70, "ball green pct: %2.2f" , ballGreenPct);
    //if (ballGreenPct > 0.4) continue;

    float dist = (ball.loc - cand).getMagnitude();
    int frames = getframe() - ball.frameLastSeen;
    // Something is wrong with computation of frames...
    if (frames > 0)
      c.velocity = dist / frames;
    else 
      c.velocity = 0;
#ifdef TOOL
    c.velocity = 0; // Velocity can be problematic when running logs, especially when log frames aren't sequential
#endif

    //    tlog(70, "cur frame %d lastSeenFrame %d", getframe(), ball.frameLastSeen);
    //    tlog(70, "ball loc (%f,%f) candLoc (%f,%f) dist %f, frames %d, initialized %d", ball.loc.x, ball.loc.y, cand.x, cand.y, dist, frames, ballInitialized);
    tlog(70, "ball velocity: %2.2f", c.velocity);
   
    Point2D offField(fabs(c.absPosition.x), fabs(c.absPosition.y));
    offField.x = offField.x > HALF_FIELD_X ? offField.x - HALF_FIELD_X : 0;
    offField.y = offField.y > HALF_FIELD_Y ? offField.y - HALF_FIELD_Y : 0;
    float distanceFromField = offField.getMagnitude();
    tlog(70, "ball distance from field: %2.2f", distanceFromField);

#ifdef TOOL
    distanceFromField = 0.0;
#endif
    
    float height = c.relPosition.z;
    tlog(70, "ball height: %2.2f", height);
   
    float prob = estimator_.getLikelihood(
      belowGreenPct,
      ballGreenPct,
      height,
      c.kwDistanceDiscrepancy,
      distanceFromField,
      c.velocity,
      triangle.score
    );
    //estimator_.printLast();
    if(prob < 0.4) {  // 0.65
      tlog(70, "threw out ball candidate because of probability: %2.2f < 0.4", prob);
      continue;
    }
    c.confidence = prob;
    c.triangleScore = triangle.score;
    estimator_.logLast(70,textlogger);

    if(prob > curBestScore) {
      curBestScore = prob; //triangle.score;
      bestCandidateIndex = candidates_.size();
      best = const_cast<ROI*>(&roi);
      best_mat = mat;
      best_triangle = triangle;
    }
    candidates_.push_back(c);
  }
#ifdef TOOL
  vblocks_.roi->ball_rois_.insert(vblocks_.roi->ball_rois_.begin(), rois.begin(), rois.end());
#elif LOG_NEGATIVES
  for(int i = 0; i < rois.size(); i++) {
    if(i == bestCandidateIndex) continue;
    vblocks_.roi->ball_rois_.push_back(rois[i].clone());
  }
#endif

  if(bestCandidateIndex != -1) {

#ifdef TOOL
    tlog(70, "-----------------------");
    // Log information about the candidates selected
    for (int i = 0; i < candidates_.size(); i++){
      tlog(70, "candidate %d confidence %3.2f triangle_score %3.2f", i, candidates_[i].confidence, candidates_[i].triangleScore);
    }
#endif


    auto& c = candidates_[bestCandidateIndex];
    tlog(70, "setting best candidate to index %i", bestCandidateIndex);
   
    if(camera_ == Camera::BOTTOM) {
      refineBallLocation(c, best, best_mat, best_triangle);
    }
    if(!LOG_NEGATIVES && VisionCore::isStreaming() && vblocks_.roi->log_block && best != nullptr) {
      *best = ROI(
        std::max(0, (int)(c.centerX - c.radius)),
        std::min(iparams_.width-1, (int)(c.centerX + c.radius)),
        std::max(0, (int)(c.centerY - c.radius)),
        std::min(iparams_.height-1, (int)(c.centerY + c.radius)),
        camera_);
      best->xstep = best->ystep = 1;
      color_segmenter_.extractMat(*best, ColorSegmenter::Source::HighresGrayscale);
      cv::Mat resized;
      cv::resize(best->mat, resized, cv::Size(32, 32));
      best->mat = resized;
      vblocks_.roi->ball_rois_ = { best->clone() };
    }
    float prediction;
     //TODO: use both top and bottom classifier
    if(classifier_ != nullptr) {
      int diff = getframe() - ball.frameLastSeen;
#ifdef TOOL
      diff = 31;    // Force classifier to run on tool so we can see if it works!
#endif
      if(diff > 30 || c.velocity > 1000) {
//        VisionTimer::Start(70, "BallDetector(%s)::deep classifier", camera_);
        *best = ROI(
          std::max(0, (int)(c.centerX - c.radius)),
          std::min(iparams_.width-1, (int)(c.centerX + c.radius)),
          std::max(0, (int)(c.centerY - c.radius)),
          std::min(iparams_.height-1, (int)(c.centerY + c.radius)),
          camera_);
        tlog(70, "reset best roi to %s", *best);
        best->xstep = best->ystep = 1;
        color_segmenter_.extractMat(*best, ColorSegmenter::Source::HighresGrayscale);
        cv::Mat resized;
        cv::resize(best->mat, resized, cv::Size(32, 32));
        best->mat = resized;
        bool result = classifier_->classify(best->mat, prediction);  // extracts features and computes SVM prediction
//        VisionTimer::Stop("BallDetector(%s)::deep classifier", camera_);
#ifdef TOOL
        vblocks_.roi->ball_rois_.push_back(best->clone());
        tlog(70, "classifier outputs: %s", classifier_->predict(best->mat));
#endif
        tlog(70, "diff %i, checking attribute detector result: %2.2f (%s)", 
          diff, prediction, result ? "true" : "false"
        );
        if(!result) {
          bestCandidateIndex = -1;
        }
      }
    }
//    printf("Static ball\n");
    if (bestCandidateIndex != -1){
      setBest(c);
    }
  }
  if (!getball().seen && findMovingBall()){
    // TODO: For now, just take the last candidate found. But these should be filtered.
    // NOTE: The first few elements of the candidates list could possibly contain rejected candidates from the previous ball detector. 
    auto& c = candidates_[candidates_.size()-1];
    // Don't trust this detector past 4 meters
    if (c.groundDistance <= 4000){
 //     printf("Moving ball\n");
      setBest(c);
    }
    else {
      tlog(75, "Ball too far (%f). Throwing out", c.groundDistance);
    }
  }
  
  counter_++;
  VisionTimer::Stop("BallDetector(%s)::findBall", camera_);
}


bool BallDetector::findMovingBall() {

  VisionTimer::Start(70, "BallDetector(%s)::findMovingBall()", camera_);
  tlog(75, "Looking for moving balls in %s camera", camera_);

  bool foundBall = false;

  blob_detector_.resetOrangeBlobs();
  blob_detector_.formOrangeBlobs();
  blob_detector_.calculateOrangeBlobData();


  // merge orange blobs
//  uint16_t mergeIndex[MAX_ORANGE_BLOBS];
//  blob_detector_.mergeHorizontalBlobs(c_ORANGE, mergeIndex, MAX_ORANGE_BLOBS);

  std::vector<Blob*> blobs;

  for(unsigned int i = 0; i < horizontalBlob[c_ORANGE].size(); i++) {
    Blob* blob = &horizontalBlob[c_ORANGE][i];
//    if(mergeIndex[i] == MAX_ORANGE_BLOBS) {
      blobs.push_back(blob);
//      tlog(34, "merged blob %i: (xi:xf,yi:yf) = (%i:%i,%i:%i)", i, blob->xi, blob->xf, blob->yi, blob->yf);
//    }
  }

  std::vector<BallCandidate> candidates;

  const float squareRatio = 1.5; 
  for(int i = 0; i < blobs.size(); i++){
    
    Blob* blob = blobs[i];
    
    // Throw out all blobs that aren't square-ish
    float squareTest = 1.0 * blob->dx / blob->dy;
    if (squareTest > squareRatio || squareTest < (1.0/squareRatio)){
      blob->invalid = true;
      continue;
    }
    else
      blob->invalid = false;


    // Fit circles to ball candidates
    BallCandidate candidate = formBallCandidateFromBlob(blob);

    tlog(75, "created ball candidate at (%f,%f) with radius %f", candidate.centerX, candidate.centerY, candidate.radius);


    // Check that green surrounds all sides
    if (!checkSurroundedByGreen(candidate)){
      tlog(75, "  failed surrounded by green check");
      continue;
    }

    candidates.push_back(candidate);

  }


  // Apply a ton of checks to make sure this is actually a ball.
  BallCandidate* best = evaluateMovingBallCandidates(candidates); 
  if (best){
    candidates_.push_back(*best);
    foundBall = true;
//    printf("Found ball\n");
  }

  // Check that inside has some undefined for the pentagons, so that we don't mix up with penalty cross

  VisionTimer::Stop("BallDetector(%s)::findMovingBall()", camera_);
  return foundBall;

}


bool BallDetector::checkSurroundedByGreen(BallCandidate& candidate){

  bool surroundedByGreen = true;

  // We use the default step sizes because the area under the ball shouldn't be high-res scanned
  int hstep = 1 << iparams_.defaultHorizontalStepScale, vstep = 1 << iparams_.defaultVerticalStepScale;

  const int padding = vstep * 3;
  const float green_thresh = 0.5;

  // Check above
  int ymin = candidate.centerY - candidate.radius - padding;
  int ymax = candidate.centerY - candidate.radius;
  int xmin = candidate.centerX - candidate.radius;
  int xmax = candidate.centerX + candidate.radius;
  float aboveGreen = greenInBox(xmin, xmax, ymin, ymax);
  surroundedByGreen &= ( aboveGreen > green_thresh);

  // left
  ymin = candidate.centerY - candidate.radius;
  ymax = candidate.centerY + candidate.radius;
  xmin = candidate.centerX - candidate.radius - padding;
  xmax = candidate.centerX - candidate.radius;
  float leftGreen = greenInBox(xmin, xmax, ymin, ymax);
  surroundedByGreen &= ( leftGreen > green_thresh);


  // right
  ymin = candidate.centerY - candidate.radius;
  ymax = candidate.centerY + candidate.radius;
  xmin = candidate.centerX + candidate.radius;
  xmax = candidate.centerX + candidate.radius + padding;
  float rightGreen = greenInBox(xmin, xmax, ymin, ymax);
  surroundedByGreen &= ( rightGreen > green_thresh);

  tlog(75, "Green around check: above %f left %f right %f thresh %f", aboveGreen, leftGreen, rightGreen, green_thresh);

  // we have a test for below later

  return surroundedByGreen;
}

float BallDetector::greenInBox(int xmin, int xmax, int ymin, int ymax){
  int hstep = 1 << iparams_.defaultHorizontalStepScale, vstep = 1 << iparams_.defaultVerticalStepScale;

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
  float pct = (float)(green) / total;
  if(!total)
    pct = 0;
  return pct;
}


bool BallDetector::intersectsShoulder(BallCandidate* candidate) {
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;
  auto ballPos = candidate->relPosition;
  auto camPos = vblocks_.body_model->abs_parts_[camera].translation;
  auto ray = ballPos - camPos;
  auto direction = ray / ray.abs();
  auto lshoulderPos = vblocks_.body_model->abs_parts_[BodyPart::left_bicep].translation;
  auto rshoulderPos = vblocks_.body_model->abs_parts_[BodyPart::right_bicep].translation;
  float radius = 40;
  auto shoulder = lshoulderPos;
  if(vblocks_.joint->values_[HeadPan] < 0)
    shoulder = rshoulderPos;
  float discriminant = powf(direction * (camPos - shoulder), 2) - (camPos - shoulder).squareAbs() + powf(radius, 2);
  bool intersects = discriminant >= 0;
  tlog(34, "camera at %2.f,%2.f,%2.f; ball at %2.f,%2.f,%2.f; shoulder at %2.f,%2.f,%2.f",
    camPos.x, camPos.y, camPos.z,
    ballPos.x, ballPos.y, ballPos.z,
    shoulder.x, shoulder.y, shoulder.z
  );
  tlog(34, "candidate at %2.f,%2.f %s %s shoulder (disc %2.2f)", candidate->centerX, candidate->centerY, intersects? "intersects" : "does not intersect", (shoulder == lshoulderPos) ? "left" : "right", discriminant);
  //printf("candidate at %2.f,%2.f %s %s shoulder (disc %2.2f)\n", candidate->centerX, candidate->centerY, intersects? "intersects" : "does not intersect", (shoulder == lshoulderPos) ? "left" : "right", discriminant);
  if(discriminant >= 0) return true;
  return false;
}



BallCandidate* BallDetector::evaluateMovingBallCandidates(std::vector<BallCandidate>& candidates) {
  tlog(75, "processing %i candidates", candidates.size());
  if(candidates.size() == 0) return false;
  Position ballRelPos(getball().relPos.x, getball().relPos.y, 0);
  float ballDistance = cmatrix_.groundDistance(ballRelPos);
  bool ballInitialized = (getball().frameLastSeen > 0);
  const float confThreshold = .25;
  float maxV = 100;
  float maxD = 100;
  // Scale max velocity with expected distance since positional uncertainty increases w/ range
  if(ballInitialized) {
    maxV = std::max(100.0f, ballDistance * .1f);
    maxD = std::max(100.0f, ballDistance * .2f);
  }
  movingball_estimator_.setStdDev(0.2f, 0.2f, 0.15f, 0.2f, 0.15f, 260.0f, 0.2f); //, maxD, maxV);
  penaltyball_estimator_.setStdDev(0.1f, 0.4f, 0.2f, 260.0f, 0.2f, maxD);
  for(uint16_t i = 0; i < candidates.size(); i++) {
    tlog(75, "processing candidate %i", i);
    BallCandidate* candidate = &candidates[i];
    Blob* blob = candidate->blob;
    int bw = blob->xf - blob->xi, bh = blob->yf - blob->yi;
    if(

        // TODO: correctpixelratio is incorrect with the orange blob hack.

        // Look at the blobs for these since half circle fitting can alter the height/width calculations.
        bw < 2 ||
        bh < 2 //||
//        candidate->blob->correctPixelRatio < .25 // These are all over the place so they don't really fit into the Gaussian model
    ) { 
      tlog(75, "threw out ball candidate because: xf - xi = %i (min 2), yf - yi = %i (min 2), pixel ratio %2.2f (min .25)", bw, bh, candidate->blob->correctPixelRatio);
      candidate->confidence = 0; continue; 
    }
    
    // The bottom camera can only see the robot's shoulder between 60 and 120 degrees on either side, so throw out all candidates here
    if(intersectsShoulder(candidate)) {
        tlog(75, "threw out ball candidate for being on robot shoulder");
        candidate->confidence = 0;
        continue;
    }

    
    float whitePct;
    float undefPct;
    float ballPct;
    checkColorsInCircleFit(candidate, whitePct, undefPct, ballPct);
    
    float belowGreenPct = camera_ == Camera::TOP ? checkBelowGreenPct(candidates[i], false) : 1.0;
    float circleFit = candidate->stddev;
    float height = candidate->relPosition.z;


    Point2D pa = Point2D(abs(candidate->absPosition.x), abs(candidate->absPosition.y));
    float xd = (pa.x > HALF_GRASS_X ? pa.x - HALF_GRASS_X : 0), yd = (pa.y > HALF_GRASS_Y ? pa.y - HALF_GRASS_Y : 0);
    //printf("pa: %2.2f,%2.2f, xd: %2.2f, yd: %2.2f\n", pa.x, pa.y, xd, yd);
    float distanceFromField = sqrt(xd * xd + yd * yd);

    Position diff = ballRelPos - candidate->relPosition;
    int dt = getframe() - getball().frameLastSeen;
    double dx = abs(diff.x), dy = abs(diff.y);
    float v = sqrt(dx * dx + dy * dy) / dt;

    // HACK: if the ball hasn't been seen in 100 frames
    // and it's within a 20cm box around the penalty cross
    // then this is probably just a false positive so we 
    // throw it out.
    // TODO: filter out cross false positives
    if(dt > 100 && pa.y < 200 && fabs(pa.x - PENALTY_CROSS_X) < 200) {
      continue;
    }
#ifdef TOOL
    v = 0; // Velocity can be problematic when running logs, especially when log frames aren't sequential
#endif
    if(!ballInitialized || dt == 0) v = 0;
    if(vblocks_.game_state->state() == TESTING) distanceFromField = 0; // Don't turn this off for SET or PLAYING or the robots will literally explode - JM 06/01/15

    double prob;
    tlog(75, "Checking candidate %i: %i,%i to %i,%i", i, blob->xi, blob->yi, blob->xf, blob->yf);
    if (vblocks_.game_state->isPenaltyKick && vblocks_.robot_state->role_ == KEEPER){
      belowGreenPct = camera_ == Camera::TOP ? checkBelowGreenPct(candidates[i], true) : 1.0;
      prob = penaltyball_estimator_.getLikelihood(
        ballPct,
        belowGreenPct,
        circleFit,
        height,
        candidate->kwDistanceDiscrepancy,
        distanceFromField
       // v
      );
      penaltyball_estimator_.logLast(75,textlogger);
//      if (prob > 0.15) printf("ball: %f, below %f, circle %f, height %f, kw %f, dist %f, v %f ===== prob %f\n", ballPct, belowGreenPct, circleFit, height, candidate->kwDistanceDiscrepancy, distanceFromField, v, prob);
    }
    else{
      prob= movingball_estimator_.getLikelihood(
        whitePct,
        undefPct,
        ballPct,
        belowGreenPct,
        circleFit,
        height,
        candidate->kwDistanceDiscrepancy
//        distanceFromField,
//        v
      );
      movingball_estimator_.logLast(75,textlogger);
    }

//    if (prob > confThreshold){
//      printf("BALL CANDIDATE (%f): wp %f up %f bp %f bg %f cf %f h %f kw %f df %f v %f\n", prob, whitePct, undefPct, ballPct, belowGreenPct, circleFit, height, candidate->kwDistanceDiscrepancy, distanceFromField, v);
//    }

    candidate->confidence = prob;
  }
  BallCandidate* best = NULL; float bestConf = 0;
  for(uint16_t i = 0; i < candidates.size(); i++) {
    BallCandidate* candidate = &candidates[i];
    if(candidate->confidence < confThreshold){
      tlog(75, "ball candidate %i thrown out for having confidence %f lower than threshold %f", i, candidate->confidence, confThreshold);
      continue;
    }
    // Final check to get rid of balls not on the field
    // TODO: this needs to be changed anyways to be smaller radius around
//    if (camera_ == Camera::TOP && checkGreenAround(candidate) < 0.25){
//      tlog(34, "ball thrown out for not having enough green around");
//      continue;
//    }
    if(candidate->confidence > bestConf) {
      best = candidate;
      bestConf = candidate->confidence;
    }
  }
  if(!best) return NULL;
  //printf("best position: %2.2f,%2.2f\n", best->absPosition.x, best->absPosition.y);
  return best;
}








void BallDetector::checkColorsInCircleFit(BallCandidate* candidate, float &whitePct, float &undefPct, float &pctBall){
  // check pixels inside circle radius
  // whats the length of one side of the largest square inside the circle
//  float squareWidth = sqrtf((4.0*candidate->radius*candidate->radius)/2.0);

  int hstep, vstep;
  color_segmenter_.getStepSize(hstep,vstep);
/*  int roundX = hstep;
  int roundY = vstep;

  int startX = ((int)((candidate->centerX - squareWidth/2.0)/roundX))*roundX;
  int startY = ((int)((candidate->centerY - squareWidth/2.0)/roundY))*roundY;
  int endX = candidate->centerX + squareWidth/2.0;
  int endY = candidate->centerY + squareWidth/2.0;
  if (startX < 0) startX = 0;
  if (startY < 0) startY = 0;
  if (endX >= iparams_.width) endX = iparams_.width-1;
  if (endY >= iparams_.height) endY = iparams_.height-1;
*/  
  
  
  int totalPixelsChecked = 0;
  int numBallPix = 0;

  /////

  int ymin = candidate->centerY - candidate->radius;
  int ymax = candidate->centerY + candidate->radius;
  int xmin = candidate->centerX - candidate->radius;
  int xmax = candidate->centerX + candidate->radius;
  
  // Make sure indices start on pixels that are classified and within image boundaries
  ymin -= ymin % vstep;
  ymax += vstep - ymax % vstep;
  xmin -= xmin % hstep;
  xmax += hstep - xmax % hstep;

  xmin = std::max(xmin, 0);
  xmax = std::min(xmax, iparams_.width - 1);
  ymin = std::max(ymin, 0);
  ymax = std::min(ymax, iparams_.height - 1);

  float radiusSquared = candidate->radius * candidate->radius;



/////




  int white = 0;
  int undef = 0;
  int topWhite = 0;
  int bottomWhite = 0;

  for (int x = xmin; x <= xmax; x+=hstep){
    
    float xoff = x - candidate->centerX;
    xoff *= xoff;
    
    for (int y = ymin; y <= ymax; y+=vstep){
      
      float yoff = y - candidate->centerY;
      yoff *= yoff;

      if (xoff + yoff <= radiusSquared){
      totalPixelsChecked++;
      int c = getSegPixelValueAt(x, y);
      // We assume a light source located above. Due to the 3D shape of the ball, the top portion will be brighter than the bottom. On the other hand, lines are flat, so the lighting will be consistent over the entire object. We use robot white as a crude way of identifying shadow regions.
      if (c == c_WHITE || c == c_ROBOT_WHITE || c == c_UNDEFINED || c == c_BLUE) numBallPix++;
      
      if (c == c_WHITE || c == c_ROBOT_WHITE) white++;
      if (c == c_WHITE && y < (ymin+((ymax-ymin)/3.0))) topWhite++;
      if (c == c_ROBOT_WHITE && y >= (ymin+((ymax-ymin)/3.0))) bottomWhite++;
      if (c == c_BLUE) undef++;
      }
    }
  }
//  white = topWhite + bottomWhite;
  if (totalPixelsChecked == 0) {
    whitePct = 0;
    undefPct = 0;
    pctBall = 0;
    //return 0;
  }

  // This is all really hacky.

  undefPct = (float)undef/totalPixelsChecked;
//  if (undefPct < 0.05){         // 0.05
//    tlog(75, "Undef pct was %f < 0.10. Clearing", undefPct);
//    undefPct = -1.0;               // HACK 
  
  // We will let the undef criteria go if the gray criteria is satisfied. Sometimes when the shutter speed is slow and there is not a lot of light, a moving ball will just appear as gray and white.

    tlog(75, "Checking undef in ball: %f", undefPct);

    if (undefPct > 0.05){
      white = std::max(white, topWhite+bottomWhite);
    }
    else {
    white = topWhite + bottomWhite;
//    undefPct = 0.15;

      // Hard constraints. Want at least 10% of each. Otherwise, it could be a line 
      if ((float)bottomWhite/(2.0 * totalPixelsChecked / 3.0) < 0.1){
        white = 0;
        tlog(75, "Not enough gray found on bottom. Found %f. Clearing white pct", (float)bottomWhite/(2.0 * totalPixelsChecked / 3.0));
      }
      if ((float)topWhite/(totalPixelsChecked / 3.0) < 0.1){
        white = 0;
        tlog(75, "Not enough white found on top. Found %f. Clearing white pct", (float)topWhite/(totalPixelsChecked / 3.0) );
      }

    }

//    undefPct = 0.2;


//  }
  whitePct = (float)white/totalPixelsChecked;
  pctBall = (float)numBallPix / (float)totalPixelsChecked;
 


//  printf("Ball white perc %f, undef perc %f, ballPix perc %f\n", whitePct, undefPct, pctBall);

//  return pctBall;
}


void BallDetector::getBlobContour(Blob* blob, uint16_t *xinit, uint16_t *xfinal) {
  int hscale, vscale, hstep, vstep;
  color_segmenter_.getStepScale(hscale,vscale);
  color_segmenter_.getStepSize(hstep,vstep);
  // initialize output
  for (uint16_t y = 0; y < iparams_.height; y += vstep) {
    xinit[y >> vscale] = (uint16_t)-1; xfinal[y >> vscale] = 0; }

  // insert first blob's contour
  for (uint16_t i = 0; i < blob->lpCount; i++) {
    uint16_t index = blob->lpIndex[i] & 0xffff;
    uint16_t y = blob->lpIndex[i] >> 16; // index
    VisionPoint *lp = &color_segmenter_.horizontalPoint[c_ORANGE][y][index];
    xinit[y >> vscale] = std::min(xinit[y >> vscale], lp->xi);    // coord
    xfinal[y >> vscale] = std::max(xfinal[y >> vscale], lp->xf);
    }  // coord
}

/**
 * Attempts to fit a perfect circle on the points of the given ball candidate.
 */
void BallDetector::fitCircleToPoints(
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


BallCandidate BallDetector::formBallCandidateFromBlob(Blob* blob){
  int hscale, vscale;
  color_segmenter_.getStepScale(hscale,vscale);
  int size = iparams_.height >> vscale;
  // get ball candidate's contour
  uint16_t xi[size], xf[size];
  getBlobContour(blob, xi, xf);

  // format full, left, right contour for circle fitting
  uint16_t contFX[iparams_.height     ], contFY[iparams_.height     ]; // full
  uint16_t contFCount = 0;
  uint16_t contLX[size], contLY[size]; // left
  uint16_t contLCount = 0;
  uint16_t contRX[size], contRY[size]; // rite
  uint16_t contRCount = 0;
  const Blob *bl = blob;
  for (uint16_t y = 0; y < size; y++) {
    if (xi[y] == (uint16_t)-1 && xf[y] == 0) continue;
    if (bl->xi > 8) {
      contFX[contFCount] = contLX[contLCount] = xi[y];
      contFY[contFCount] = contLY[contLCount] = y << vscale;
      contFCount++; contLCount++; 
    }
    if (bl->xf < iparams_.width - 8) {
      contFX[contFCount] = contRX[contRCount] = xf[y];
      contFY[contFCount] = contRY[contRCount] = y << vscale;
      contFCount++; contRCount++; 
    }
  }

  // only consider upper half of the contour if too close to body
  if (bl->dx > 40 && bl->yi > size) {
    contFCount = 3 * contFCount / 5;
    contLCount = 3 * contLCount / 5;
    contRCount = 3 * contRCount / 5; }

  // fit circle
  float cxF, cyF, rF, sdF;
  for(int i = 0; i < contFCount; i++) {
    //std::cout << "contfx,contfy: (" << contFX[i] << "," << contFY[i] << ")\n";
  }                              // full
  fitCircleToPoints(contFX, contFY, contFCount, &cxF, &cyF, &rF, &sdF);
  BallCandidate candidate;
  //std::cout << "fit points 1: (cx,cy), r, sd: " << "(" << cxF << "," << cyF << "), " << rF << ", " << sdF << "\n";
  if (2 * bl->dy > 3 * bl->dx) {
    float cxL, cyL, rL, sdL;                                       // left
    fitCircleToPoints(contLX, contLY, contLCount, &cxL, &cyL, &rL, &sdL);
    float cxR, cyR, rR, sdR;                                       // rite
    fitCircleToPoints(contRX, contRY, contRCount, &cxR, &cyR, &rR, &sdR);

    // set final ball candidate circle fit values
    //const uint16_t cxBlob = (bl->xi + bl->xf) >> 1;
    if (cxR > bl->xf) {
      tlog(76,  "<<< LEFT contour chosen !! >>>");
      candidate.centerX = cxL;
      candidate.centerY = cyL;
      candidate.radius = rL;
      candidate.width = 2 * rL;
      candidate.stddev = sdL;
    } else if (cxL < bl->xi) {
      tlog(76,   "<<< RITE contour chosen !! >>>");
      candidate.centerX = cxR;
      candidate.centerY = cyR;
      candidate.radius = rR;
      candidate.width = 2 * rR;
      candidate.stddev = sdR;
    }
  } else {
    tlog(76,  "<<< FULL contour chosen: (cx,cy) = (%f,%f), rad = %f, stddev = %f !! >>>", cxF, cyF, rF, sdF);
    candidate.centerX = cxF;
    candidate.centerY = cyF;
    candidate.radius = rF;
    candidate.stddev = sdF;
    candidate.width = blob->dx;
  }
  candidate.blob = blob;
  candidate.height = blob->dy;
  
  // TODO: Sanmit
  float directDistance = getDirectDistance(candidate);
//  Position p = cmatrix_.getWorldPosition(candidate.centerX, candidate.centerY, BALL_RADIUS);
//  float directDistance = cmatrix_.directDistance(p);
  
  candidate.relPosition = cmatrix_.getWorldPositionByDirectDistance(candidate.centerX, candidate.centerY, directDistance);
  candidate.groundDistance = cmatrix_.groundDistance(candidate.relPosition);
  candidate.relPosition.z -= BALL_RADIUS;
  candidate.valid = true;
  Pose2D self(getself().orientation, getself().loc.x, getself().loc.y); // release
  Pose2D cand = Pose2D(candidate.relPosition.x, candidate.relPosition.y).relativeToGlobal(self);
  candidate.absPosition = Position(cand.translation.x, cand.translation.y, candidate.relPosition.z);
  return candidate;
}

float BallDetector::checkBallColor(BallCandidate &candidate) {
  int hstep = 1 << iparams_.defaultHorizontalStepScale, vstep = 1 << iparams_.defaultVerticalStepScale;

  int ymin = candidate.centerY - candidate.radius;
  int ymax = candidate.centerY + candidate.radius;
  int xmin = candidate.centerX - candidate.radius;
  int xmax = candidate.centerX + candidate.radius;
  
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
  int white = 0, undefined = 0; 
  float radiusSquared = candidate.radius * candidate.radius;

  for (int x = xmin; x <= xmax; x += hstep){
    
    float xoff = x - candidate.centerX;
    xoff *= xoff;

    for (int y = ymin; y <= ymax; y += vstep){
      float yoff = y - candidate.centerY;
      yoff *= yoff;
      if (xoff + yoff <= radiusSquared){
        int c = getSegPixelValueAt(x, y);
        total++;
        green += (c == c_FIELD_GREEN);
        white += (c == c_WHITE || c == c_ROBOT_WHITE);
        undefined += (c == c_UNDEFINED);
      }
    }
  }

  float pct = (float)(green) / total;


//  if(!total && ymin >= iparams_.height)
  // We can't see below the ball due to the height, so we assume 1.
//    pct = 1;

//  tlog(43, "check from x: %i, %i, and y: %i, %i, hstep: %i, vstep: %i, total: %i, greenwhite: %i, pct: %5.3f orange %i black %i", xmin, xmax, ymin, ymax, hstep, vstep, total, green + white, pct, orange, black);

  return pct;
}

/** Check the percantage of green below ball. */
float BallDetector::checkBelowGreenPct(BallCandidate &candidate, bool useWhite) {
  // We use the default step sizes because the area under the ball shouldn't be high-res scanned
  int hstep = 1 << iparams_.defaultHorizontalStepScale, vstep = 1 << iparams_.defaultVerticalStepScale;

  int ymin = candidate.centerY + candidate.radius;
  int ymax = candidate.centerY + 3*candidate.radius;
  int xmin = candidate.centerX - candidate.radius;
  int xmax = candidate.centerX + candidate.radius;
  
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
      if (useWhite){
        green += (c == c_FIELD_GREEN || c == c_WHITE);
      }
      else{
        green += (c == c_FIELD_GREEN);
      }
    }
  }

  float pct = (float)(green) / total;


  if(!total && ymin >= iparams_.height)
  // We can't see below the ball due to the height, so we assume 1.
    pct = 1;

//  tlog(43, "check from x: %i, %i, and y: %i, %i, hstep: %i, vstep: %i, total: %i, greenwhite: %i, pct: %5.3f orange %i black %i", xmin, xmax, ymin, ymax, hstep, vstep, total, green + white, pct, orange, black);

  // Also check using field edges
  // TODO: This is a temporary hack. We should really put a function in field edge detector to calculate whether points are above or below. 
  int projectedX = candidate.centerX - (static_cast<int>(candidate.centerX) % hstep);
  projectedX = std::max(0, projectedX);
  int projectedIndex = ((iparams_.width - projectedX)/ hstep) - 1;
  if (field_edge_detector_.hullPointCands[projectedIndex].hullY < candidate.centerY){
    pct = 1;
//    printf("Checking: %f \n", field_edge_detector_.hullPointCands[projectedIndex].x);
  }

  return pct;
}

/*
 * This function extracts black blobs from a ROI and applies a series of heuristic tests to
 * decide if the ROI contains a ball. This function also localizes the ball if one is detected.
 * The tests are:
 *    1. blob count - need at least three blobs
 *    2. relative blob size - the largest of the three blobs can't be too much larger than the smallest of the three blobs
 *    3. angle between blobs - three blobs form a triangle. The large angle of the triangle should be < 90 degrees
 *    4. float test - Assuming three blobs define a ball, we can solve for the elevation of the ball.
 *                    Make sure it's not too far above the ground or too far below.
 */

template<typename T, int N, typename = std::enable_if_t<N == 1>, typename = void>
std::vector<std::vector<T>> generateSubsets(const std::vector<T>& values) {
  std::vector<std::vector<T>> subsets;
  for(const auto& v : values)
    subsets.push_back({v});
  return subsets;
}

template<typename T, int N, typename = std::enable_if_t<N != 1>>
std::vector<std::vector<T>> generateSubsets(std::vector<T> values) {
  std::vector<std::vector<T>> subsets;
  if(values.size() >= N) {
    auto v = values.back();
    values.pop_back();
    auto subsubs = generateSubsets<T,N-1>(values);
    for(auto& s : subsubs)
      s.push_back(v);
    subsets.insert(subsets.end(), subsubs.begin(), subsubs.end());
    subsubs = generateSubsets<T,N>(values);
    subsets.insert(subsets.end(), subsubs.begin(), subsubs.end());
  }
  return subsets;
}

bool BallDetector::shouldMergeBlobs(BallBlob& b1, BallBlob& b2) {
  if (b1.color != b2.color) {
    return false;
  }

  // if bounding boxes are too far, return false
  if (b1.minX > b2.maxX
      || b2.minX > b1.maxX
      || b1.minY > b2.maxY + 1
      || b2.minY > b1.maxY + 1) {
    return false;
  }

  // check if any two pixels between blob1 and blob2 have
  // manhattan distance == 2
  for (auto& sl1 : b1.getScanLineVector()) {
    for (auto& sl2 : b2.getScanLineVector()) {
      int ydiff = std::abs(sl1.y - sl2.y);
      switch (ydiff) {
        case 1:
          if (sl1.end +1 == sl2.start || sl1.start == sl2.end + 1) {
            return true;
          }
          break;
        case 2:
          if (!(sl1.end < sl2.start || sl1.start > sl2.end)) {
            return true;
          }
          break;
      }
    }
  }

  return false;
}

BlobTriangle BallDetector::blobTests(cv::Mat& mat, const ROI& roi, int xstep, int ystep, bool highres) {  
//  VisionTimer::Start(70, "BallDetector(%s)::computeBlobs", camera_);
  std::vector<std::vector<ScanLine>> lines(mat.rows);
  std::vector<BallBlob> blobs = computeBlobs(mat, lines);
//  VisionTimer::Stop("BallDetector(%s)::computeBlobs", camera_);

//  VisionTimer::Start(70, "BallDetector(%s)::mergeBlobs", camera_);
  for (int i = 0; i < blobs.size()-1; i++) {
    BallBlob& blob1 = blobs[i];
    for (int j = i + 1; j < blobs.size(); j++) {
      BallBlob& blob2 = blobs[j];
      if (shouldMergeBlobs(blob1, blob2)) {
        // merge blob 2 into blob1 and then delete blob 2
        blob1.merge(blob2, 1);
        tlog(70, "merged blob %i into blob %i", blob2.id, blob1.id);
        // remove blob2
        std::swap(blob2, blobs.back());
        blobs.pop_back();
        // reduce j index because we removed from the list
        j--;
      }
    }
  }
//  VisionTimer::Stop("BallDetector(%s)::merge blobs", camera_);
  
//  VisionTimer::Start(70, "BallDetector(%s)::filterBlobs", camera_);
  std::vector<BallBlob> badBlobs;
  blobs = filterBlobs(blobs, badBlobs, roi, xstep, ystep, highres);
//  VisionTimer::Stop("BallDetector(%s)::filterBlobs", camera_);

#ifdef TOOL
  if(DEBUG_BLOB) {
    std::string filepath = util::ssprintf("%s/BlobImages/blobs_%02i_%s_%02i_ball.png", util::env("NAO_HOME"), counter_, (camera_ == Camera::TOP) ? "top" : "bottom", roiCounter_);
    cv::imwrite(filepath, mat);
    filepath = util::ssprintf("%s/BlobImages/blobs_%02i_%s_%02i.png", util::env("NAO_HOME"), counter_, (camera_ == Camera::TOP) ? "top" : "bottom", roiCounter_);
    createBlobImage(mat, blobs, badBlobs, filepath);
  }
  else {
    createBlobImage(roi, blobs, badBlobs);
  }
#endif

  // blob count test: need at least three blobs to be a soccer ball
  if(blobs.size() < 3) {
    tlog(70, "fail blob count test: blobs.size() = %i < 3", blobs.size());
    return -1;
  }
  else {
    tlog(70, "pass blob count test: blobs.size() = %i >= 3", blobs.size());
  }

  float score = -1;
  if(blobs.size() == 3) {
    auto t = BlobTriangle(blobs[0], blobs[1], blobs[2]);
    //score = blobGeometryTests(blobs, 0, 1, 2, roi, xstep, ystep, centerX, centerY, radius);
    return blobGeometryTests({t}, blobs, badBlobs, roi, xstep, ystep);
  }
  else {
    VisionTimer::Wrap(88, "ball triangle subsets");
    int maxBlobs = 7;
    if(blobs.size() > maxBlobs)
      blobs.erase(blobs.begin() + maxBlobs);
    auto subsets = generateSubsets<BallBlob,3>(blobs);
    std::vector<BlobTriangle> triangles;
    for(const auto& s : subsets)
      triangles.push_back(BlobTriangle(s[0], s[1], s[2]));
    auto result = blobGeometryTests(triangles, blobs, badBlobs, roi, xstep, ystep);
    VisionTimer::Wrap(88, "ball triangle subsets");
    return result;
  }
}

/*
 * Arguments should be sorted in descending order of blob area. (blob1 is largest)
 */
 #define SIGN(x1, y1, x2, y2, x3, y3) (x1 - x3)*(y2 - y3) - (x2 - x3)*(y1 - y3)
BlobTriangle BallDetector::blobGeometryTests(std::vector<BlobTriangle>& triangles, const std::vector<BallBlob>& blobs, const std::vector<BallBlob>& badBlobs, const ROI& roi, int xstep, int ystep) {
  BlobTriangle bestT;
  for(auto& t : triangles) {
    //VisionTimer::Start(70, "BallDetector(%s)::blobGeometryTests", camera_);
    blobGeometryTests(t, blobs, badBlobs, roi, xstep, ystep);
    //VisionTimer::Stop("BallDetector(%s)::blobGeometryTests", camera_);
    if(t.score > bestT.score) {
      bestT = t;
    }
  }
  return bestT;
}

BlobTriangle BallDetector::blobGeometryTests(BlobTriangle& t, const std::vector<BallBlob>& blobs, const std::vector<BallBlob>& badBlobs, const ROI& roi, int xstep, int ystep) {
  auto &blob1 = t.b1, &blob2 = t.b2, &blob3 = t.b3;

  tlog(70, "blob geometry tests for blobs %d %d %d", blob1.id, blob2.id, blob3.id);

  // relative size test: largest blob can't be 4 time larger than smallest blob
  int threshold = 4;
  if(camera_ == Camera::TOP) {
    threshold = 8;
  }
  if(blob1.area > threshold*blob3.area) {
    tlog(70, "  fail relative size test: A1 / A3 = %2.2f < %2.2f", static_cast<float>(blob1.area) / blob3.area, threshold);
    t.score = -1;
    return -1;
  }
  else {
    tlog(70, "  pass relative size test: A1 / A3 = %2.2f < %2.2f", static_cast<float>(blob1.area) / blob3.area, threshold);
  }

  // absolute darkness test
  int darkness_threshold = 100;
  std::vector<int> intensities;
  intensities.push_back(blob1.avgPixelIntensity);
  intensities.push_back(blob2.avgPixelIntensity);
  intensities.push_back(blob3.avgPixelIntensity);
  std::sort(intensities.begin(), intensities.end());
  if(intensities[2] > darkness_threshold) {
    tlog(70, "  fail absolute darkness test: %d %d %d > %d", intensities[0], intensities[1], intensities[2], darkness_threshold);
    intensities.clear();
    intensities.push_back(blob1.getAvgIntensityWithoutLightest());
    intensities.push_back(blob2.getAvgIntensityWithoutLightest());
    intensities.push_back(blob3.getAvgIntensityWithoutLightest());
    std::sort(intensities.begin(), intensities.end());
    if(intensities[2] > darkness_threshold) {
      tlog(70, "  also fail absolute darkness test without lightestPixel: %d %d %d > %d", intensities[0], intensities[1], intensities[2], darkness_threshold);
      t.score = -1;
      return -1;
    }
    tlog(70, "  pass absolute darkness test without lightestPixel: %d %d %d > %d", intensities[0], intensities[1], intensities[2], darkness_threshold);
  }
  else {
    tlog(70, "  pass absolute darkness test: %d %d %d <= %d", intensities[0], intensities[1], intensities[2], darkness_threshold);
  }

  // angle check
  // get largest angle from largest side length of triangle.
  std::vector<float> side_lengths_squared;
  side_lengths_squared.push_back(std::pow((blob1.centroidX - blob2.centroidX)*xstep, 2) + std::pow((blob1.centroidY - blob2.centroidY)*ystep, 2));
  side_lengths_squared.push_back(std::pow((blob1.centroidX - blob3.centroidX)*xstep, 2) + std::pow((blob1.centroidY - blob3.centroidY)*ystep, 2));
  side_lengths_squared.push_back(std::pow((blob2.centroidX - blob3.centroidX)*xstep, 2) + std::pow((blob2.centroidY - blob3.centroidY)*ystep, 2));
  std::sort(side_lengths_squared.begin(), side_lengths_squared.end());
  float a2 = side_lengths_squared[0];  
  float b2 = side_lengths_squared[1];  
  float c2 = side_lengths_squared[2];  // largest side length
  if(c2 > 3*a2) {
    tlog(70, "  fail side length test a2, b2, c2 = %.4f, %.4f, %.4f", a2, b2, c2);
    t.score = -1;
    return -1;
  }
  else {
    tlog(70, "  pass side length test a2, b2, c2 = %.4f, %.4f, %.4f", a2, b2, c2);
  }
  float cos_theta = (a2 + b2 - c2)/(2*std::sqrt(a2*b2)); // Law of Cosines
  tlog(70, "  computing cos(t) for blobs at (%i,%i), (%i,%i), (%i,%i)",
    blob1.centroidX*xstep + roi.xmin, blob1.centroidY*ystep + roi.ymin,
    blob2.centroidX*xstep + roi.xmin, blob2.centroidY*ystep + roi.ymin,
    blob3.centroidX*xstep + roi.xmin, blob3.centroidY*ystep + roi.ymin
  );
  if(cos_theta < 0) {  // if angle > 90 degrees
    tlog(70, "  fail angle test: cos(t) = %2.2f < %2.2f", cos_theta, 0.0f);
    t.score = -1;
    return -1;
  }
  else {
    tlog(70, "  pass angle test: cos(t) = %2.2f > %2.2f", cos_theta, 0.0f);
  }

  // float test
  float ballWidth = BALL_RADIUS * 2; // millimeters
  int minX = min(blob1.centroidX, blob2.centroidX, blob3.centroidX)*xstep + roi.xmin;
  int maxX = max(blob1.centroidX, blob2.centroidX, blob3.centroidX)*xstep + roi.xmin;
  int minY = min(blob1.centroidY, blob2.centroidY, blob3.centroidY)*ystep + roi.ymin;
  int maxY = max(blob1.centroidY, blob2.centroidY, blob3.centroidY)*ystep + roi.ymin;
  
  t.centerX = (maxX + minX)/2;
  t.centerY = (maxY + minY)/2;
  t.radius = (maxX-minX + maxY-minY)/2.0;
  float larger_radius = t.radius/0.7;
  float smaller_radius = t.radius/1.3;

  // put ball above ground => large radius and small y
  float distance = cmatrix_.getWorldDistanceByWidth(2*larger_radius, ballWidth);
  Position position = cmatrix_.getWorldPositionByDirectDistance(t.centerX, minY, distance);
  float elevation = position.z;
  float elevation_thresh = 200.0f;
  if(elevation < -elevation_thresh) {
    tlog(70, "  fail float test: elevation = %2.2f < %2.2f", elevation, -elevation_thresh);
    t.score = -1;
    return -1;
  }
  else {
    tlog(70, "  pass float test: elevation = %2.2f >= %2.2f", elevation, -elevation_thresh);
  }
  // put ball below ground -> small radius and large y
  distance = cmatrix_.getWorldDistanceByWidth(2*smaller_radius, ballWidth);
  position = cmatrix_.getWorldPositionByDirectDistance(t.centerX, maxY, distance);
  elevation = position.z;
  if(elevation > elevation_thresh) {
    tlog(70, "  fail float test 2: elevation = %2.2f > %2.2f", elevation, elevation_thresh);
    t.score = -1;
    return -1;
  }
  else {
    tlog(70, "  pass float test 2: elevation = %2.2f <= %2.2f", elevation, elevation_thresh);
  }

  // "no other blobs in middle of soccer ball" test
  for(auto b : blobs) {
    if(b.id == blob1.id || b.id == blob2.id || b.id == blob3.id) continue;

    // check if blob is inbetween the candidate blobs.
    bool b1 = SIGN(b.centroidX, b.centroidY, blob1.centroidX, blob1.centroidY, blob2.centroidX, blob2.centroidY) <= 0;
    bool b2 = SIGN(b.centroidX, b.centroidY, blob2.centroidX, blob2.centroidY, blob3.centroidX, blob3.centroidY) <= 0;
    bool b3 = SIGN(b.centroidX, b.centroidY, blob3.centroidX, blob3.centroidY, blob1.centroidX, blob1.centroidY) <= 0;

    if((b1 == b2) && (b2 == b3)) {
      // centroid of b is inside the triangle formed by centroids of blob1, blob2, blob3.
      tlog(70, "  fail contains other (good) blob");
      t.score = -1;
      return -1;
    }
  }
  for(auto b : badBlobs) {
    if(b.id == blob1.id || b.id == blob2.id || b.id == blob3.id) continue;

    // check if blob is inbetween the candidate blobs.
    bool b1 = SIGN(b.centroidX, b.centroidY, blob1.centroidX, blob1.centroidY, blob2.centroidX, blob2.centroidY) <= 0;
    bool b2 = SIGN(b.centroidX, b.centroidY, blob2.centroidX, blob2.centroidY, blob3.centroidX, blob3.centroidY) <= 0;
    bool b3 = SIGN(b.centroidX, b.centroidY, blob3.centroidX, blob3.centroidY, blob1.centroidX, blob1.centroidY) <= 0;

    if((b1 == b2) && (b2 == b3)) {
      // centroid of b is inside the triangle formed by centroids of blob1, blob2, blob3.
      tlog(70, "  fail contains other (bad) blob");
      t.score = -1;
      return -1;
    }
  }
  tlog(70, "  pass contains other blob");
  t.score = cos_theta;
  
  tlog(70, "  passed all geometry tests with score: %f", t.score);
  
  return t;
}

float BallDetector::max(float a, float b, float c) {
  return std::max(std::max(a, b), c);
}

float BallDetector::min(float a, float b, float c) {
  return std::min(std::min(a, b), c);
}

// This function is no longer used. It's been replaced by the macro DOES_CHANGE_COLOR
// bool BallDetector::doesChangeColor(cv::Mat& mat, int row, int col, int window, int& left, int& right) {
//   left = 0;
//   for(int i = 1; i <= window; ++i) {
//     left += mat.at<unsigned char>(row, col-i);
//   }

//   right = 0;
//   for(int i = 0; i < window; ++i) {
//     right += mat.at<unsigned char>(row, col+i);
//   }
//   assert(left >= 0);
//   assert(left <= 255*window);
//   assert(right >= 0);
//   assert(right <= 255*window);

//   float percent_diff = std::abs(left - right)/((left + right)/2.0f);
//   return percent_diff > 0.25;
// }

ScanLine* BallDetector::getRoot(ScanLine& sl) {
  ScanLine* root = &sl;
  while(root->parent != NULL) {
    root = root->parent;
  }
  return root;
}

void BallDetector::merge(ScanLine& top, ScanLine& bot) {
  ScanLine* topRoot = getRoot(top);
  ScanLine* botRoot = getRoot(bot);
  if(bot.parent == NULL) {
    topRoot->children.push_back(&bot);
    bot.parent = topRoot;
  } else if (topRoot != botRoot) {
    botRoot->children.insert(botRoot->children.end(), topRoot->children.begin(), topRoot->children.end());
    topRoot->parent = botRoot;
  }
}

int BallDetector::greenPixelCount(const BallBlob& blob, const ROI& roi, int xstep, int ystep) {

//  VisionTimer::Start(70, "BallDetector(%s)::greenPixelCheck", camera_);
  // define colors
  int total = 0;
  int numGreen = 0;

  int hstep, vstep;
  color_segmenter_.getStepSize(hstep, vstep);

  for (auto& sl : blob.getScanLineVector()) {
    int y = sl.y;
    for(int x = sl.start; x < sl.end; ++x) {
  
      int x_ = x*xstep + roi.xmin;
      x_ -= x_ % hstep;

      int y_ = y*ystep + roi.ymin;
      y_ -= y_ % vstep;

      auto c = getSegPixelValueAt(x_, y_);
      numGreen += (c == c_FIELD_GREEN);
      ++total;
    }
  }

//  VisionTimer::Stop("BallDetector(%s)::greenPixelCheck", camera_);
  return numGreen;
}

std::vector<BallBlob> BallDetector::filterBlobs(std::vector<BallBlob> blobs, std::vector<BallBlob>& badBlobs, const ROI& roi, int xstep, int ystep, bool highres) {
  std::vector<BallBlob> goodBlobs;

  tlog(70, "filtering blobs");

  for(BallBlob b : blobs) {
    if(b.color != DARK) {
      continue;
    }

    tlog(70, "blob %d", b.id);

//    if(DEBUG_BLOB) {
//      std::cout << "considering blob with area = " << b.area << " at (x, y) = (" << b.centroidX << ", " << b.centroidY << ")" << std::endl;
//    }

    float sparsity_threshold = 0.4;
//    if(camera_ == Camera::TOP) {
//      sparsity_threshold = 0.5;       // Removing this might be masking the issue that the runs are getting terminated early in some cases
//    }
    if(blobTooSmall(b, roi)) {
      tlog(70, " failed blobTooSmall.");
      badBlobs.push_back(b);
    }
    else if((b.width*xstep)/(b.height*ystep) >= 4 || (b.height*ystep)/(b.width*xstep) >= 4) {
      tlog(70, " failed aspect ratio test.");
      badBlobs.push_back(b);
    }
    else if(blobTooBig(b, roi)) {
      tlog(70, " failed blobTooBig.");
      badBlobs.push_back(b);
    }
    // blob sholdn't be sparse. Should fill it's bound box by at least 40%
    else if(b.area < sparsity_threshold*b.height*b.width) {
      tlog(70, "  failed sparsity test. Area %d < (%f) * %d * %d", b.area, sparsity_threshold, b.height, b.width);
      badBlobs.push_back(b);
    }
    else if(greenPixelCount(b, roi, xstep, ystep) >= 0.75 * b.area) {     // Ideally we shouldn't be getting a lot of green, but due to light reflecting and noise, it still happens. 
      // tlog(70, "  failed green test. numGreen = %d >= %d", numGreen, b.area/2);
      tlog(70, "  failed greenPixelCount test.");
      badBlobs.push_back(b);
    }
    else {
//      if(DEBUG_BLOB) {std::cout << "passed all tests." << std::endl;}
      tlog(70, "  passed all tests");
      goodBlobs.push_back(b);
    }
  }
  // if(camera_ == Camera::BOTTOM) {
  //   std::cout << "writing seg" << std::endl;
  //   std:;string topbottom = "bottom";
  //   std::string filepath = util::ssprintf("%s/BlobImages/blobs_%s_%02i_%02i_seg.png", util::env("NAO_HOME"), topbottom, counter_, roiCounter_);
  //   cv::imwrite(filepath, seg);
  // }
  // sort by size
  std::sort(goodBlobs.begin(), goodBlobs.end());
  tlog(70, "filtering blobs, %i good, %i bad", goodBlobs.size(), badBlobs.size());

  return goodBlobs;
}

bool BallDetector::blobTooBig(BallBlob& blob, const ROI& roi) {
  int x = blob.centroidX*roi.xstep + roi.xmin;
  int y = blob.centroidY*roi.ystep + roi.ymin;
  float expectedBlobSize = cmatrix_.getExpectedCameraWidth(x, y, 80.0, 30.0);
  if (camera_ == Camera::BOTTOM) {
    expectedBlobSize = 0.1696*expectedBlobSize + 14.097;
  }

  float expectedArea = 3.14159*std::pow(expectedBlobSize/2, 2);  // area of circle
  float upperBound = 1.5*expectedArea;

  int blobArea = blob.area*roi.xstep*roi.ystep;

  return blobArea >= upperBound;
}

bool BallDetector::blobTooSmall(BallBlob& blob, const ROI& roi) {
  int x = blob.centroidX*roi.xstep + roi.xmin;
  int y = blob.centroidY*roi.ystep + roi.ymin;
  float expectedBlobSize = cmatrix_.getExpectedCameraWidth(x, y, 80.0, 30.0);     // TODO: look at what the 80 is for... or change the division amount in lowerBound? 
  if (camera_ == Camera::BOTTOM) {
    expectedBlobSize = 0.1696*expectedBlobSize + 14.097;
  }
  
  float expectedArea = 3.14159*std::pow(expectedBlobSize/2, 2);  // area of circle
  float lowerBound = expectedArea/6;

  int blobArea = blob.area*roi.xstep*roi.ystep;
  tlog(70, "blobArea = %i; expectedArea = %.4f; ratio = %.4f", 
    blobArea, expectedArea, blobArea/expectedArea);

  return blobArea <= lowerBound;
}

/*
 * Assumes the black pentagons of the ball do not touch the left and right edges of the bounding box.
 * That is, the bouding box contains some padding/margin on the left and right sides of the ball.
 */
// TODO: make sure searching around doesn't go out of bounds? Maybe the number to check behind should depend on roi sampling factor?
#define DOES_CHANGE_COLOR(mat, row, col, left, right) \
  left = mat.at<unsigned char>(row, col-2); \
  right = mat.at<unsigned char>(row, col); \
  float percent_diff = std::abs(left - right) / ((left + right) / 2.0f); \
  bool res = percent_diff > 0.30f; \
  if(camera_ == Camera::BOTTOM) { res = percent_diff > 0.20f;}


std::vector<BallBlob> BallDetector::computeBlobs(cv::Mat& mat, std::vector<std::vector<ScanLine>>& lines) {
  int window = 1;
  double changeColorTicks = 0;
  int black_threshold = 110;

  // 1. Compute Scan lines
  for (int row = 0; row < mat.rows; ++row) {
    std::vector<ScanLine>& r = lines[row];
    ScanLine sl = ScanLine();
    sl.y = row;
    sl.color = WHITE; // assume the row starts white
    sl.start = 0;

    for (int col = 2; col < mat.cols; ++col) {
      int left, right;
      DOES_CHANGE_COLOR(mat, row, col, left, right);
      if(res || (sl.color == DARK && mat.at<unsigned char>(row, col) > black_threshold)) {
        int next_color = WHITE;
        if(left > right && mat.at<unsigned char>(row, col) < black_threshold) {
          // switching from white to black
          next_color = DARK;
        }
        if(next_color == sl.color){                    
          // not really a color switch - going from white to even more white, or black to even more black
          continue;
        }

        // end the current ScanLine
        sl.end = col - window;
        r.push_back(sl);

        // if(DEBUG_BLOB) {
        //   std::cout << "changing " << (int)(sl.color) << " -> " << (int)(next_color) << " on row " << row << " at column " << col << " length = " << (sl.end - sl.start + 1) << ". left = " << left << ", right = " << right << std::endl;
        // }

        // start new ScanLine
        sl = ScanLine();
        sl.y = row;
        sl.start = col;
        sl.color = next_color;
      }
      sl.pixelIntensitySum += right;
      sl.lightestPixel = std::max(sl.lightestPixel, right);
    }

    sl.end = mat.cols - 1;
    sl.color = WHITE;
    r.push_back(sl);
  }

  // 2. merge scan lines
  for (int i = 1; i < lines.size(); ++i) {
    auto topIt = lines[i-1].begin();
    auto botIt = lines[i].begin();
    while(topIt != lines[i-1].end() && botIt != lines[i].end()) {
      ScanLine& top = *topIt;
      ScanLine& bot = *botIt;
      if(top.color == bot.color) {
        merge(top, bot);
      }
      if(top.end < bot.end) {
        ++topIt;
      } else if(bot.end < top.end) {
        ++botIt;
      } else {
        ++topIt;
        ++botIt;
      }
    }
  }

  // 3. create Blob objects
  std::vector<BallBlob> blobs;
  int id = 0;
  for (int i = 0; i < lines.size(); ++i) {
    for (auto it = lines[i].begin(); it != lines[i].end(); ++it) {
      if(it->parent == NULL) {
        auto blob = BallBlob(&*it);
        blob.id = id++;
        blobs.push_back(std::move(blob));
        tlog(70, "created blob %i at (%i,%i): [%i,%i] --> [%i,%i]",
          blob.id, blob.centroidX, blob.centroidY, blob.minX, blob.minY, blob.maxX, blob.maxY
        );
      }
    }
  }

  return blobs;
}

void BallDetector::refineBallLocation(BallCandidate& candidate, const ROI* roi, cv::Mat& mat, BlobTriangle triangle) {
  // erase black spots on ball
  eraseBlob(roi->mat, triangle.b1);
  eraseBlob(roi->mat, triangle.b2);
  eraseBlob(roi->mat, triangle.b3);

  // only run this function on the bottom camera
  // xstep and ystep should be the same
  std::vector<cv::Vec3f> circles;
  int highCanny = 400;
  double dp = 1;
  double minDist = mat.rows; // only one circle allowed
  double minRadius = candidate.radius/(roi->xstep)*0.95;
  double maxRadius = candidate.radius/(roi->ystep)*1.2;
  double accumulator_threshold = 3;

  // Rouhan's close ball (smaller version)
  double high_thresh = cv::threshold(mat, mat.clone(), 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
  highCanny = high_thresh;
  /////

  cv::HoughCircles(mat, circles, CV_HOUGH_GRADIENT, dp, minDist, highCanny, accumulator_threshold, minRadius, maxRadius);

  if(circles.size() == 1) {

    int circlePadding = 5;

    Point2D circlePoint(circles[0][0], circles[0][1]);
    float circleRadius = circles[0][2] + circlePadding;   // Add some padding in case blobs are on edge
    int blobContainmentCount = 0;
    if (Point2D(triangle.b1.centroidX, triangle.b1.centroidY).getDistanceTo(circlePoint) < circleRadius) {
      blobContainmentCount++;
    }
    if (Point2D(triangle.b2.centroidX, triangle.b2.centroidY).getDistanceTo(circlePoint) < circleRadius) {
      blobContainmentCount++;
    }
    if (Point2D(triangle.b3.centroidX, triangle.b3.centroidY).getDistanceTo(circlePoint) < circleRadius) {
      blobContainmentCount++;
    }

    if (blobContainmentCount >= 2) {
      // if the hough circle contains fewer than 2 blobs, don't use it
      candidate.centerX = circles[0][0]*roi->xstep + roi->xmin;
      candidate.centerY = circles[0][1]*roi->ystep + roi->ymin;
      candidate.radius = circles[0][2]*roi->xstep;  // xstep should equal ystep
    }
  }
}

void BallDetector::eraseBlob(cv::Mat& mat, BallBlob blob) {
  for (auto& sl : blob.getScanLineVector()) {
    for(int x = std::max(0, sl.start-1); x <= std::min(mat.cols-1, sl.end+1); ++x) {
      mat.at<unsigned char>(sl.y, x) = 200;
    }
  }
}

void BallDetector::colorBlob(cv::Mat& seg, BallBlob blob, cv::Vec3b color) {
  for (auto& sl : blob.getScanLineVector()) {
    for(int x = sl.start; x <= sl.end; ++x) {
      seg.at<cv::Vec3b>(sl.y, x) = color;
    }
  }
}

void BallDetector::createBlobImage(
    const ROI& roi, std::vector<BallBlob> goodBlobs,
    std::vector<BallBlob> badBlobs) {


  cv::Mat mat = roi.mat.clone();
  cv::cvtColor(mat, mat, CV_GRAY2RGB);
  mat.setTo(cv::Scalar(255,255,255));

  cv::Vec3b colorBlack(0,0,0);            // unmerged good blob
  cv::Vec3b colorBlue(91,204,235);        // merged good blob
  cv::Vec3b colorRed(255,0,0);            // unmerged bad blob
  cv::Vec3b colorPink(255,74,164);        // merged bad blob

  for(auto& blob : goodBlobs) {
    if (blob.wasMerged()) {
      colorBlob(mat, blob, colorBlue);
    } else {
      colorBlob(mat, blob, colorBlack);
    }
  }
  for(auto& blob : badBlobs) {
    if (blob.wasMerged()) {
      colorBlob(mat, blob, colorPink);
    } else {
      colorBlob(mat, blob, colorRed);
    }
  }

  // Resize to fit in original image space
  cv::resize(mat, mat, cv::Size(), roi.xstep, roi.ystep);
  
  // Copy into blob debug image
  mat.copyTo(blobDebugImg(cv::Rect(roi.xmin, roi.ymin, roi.xmax - roi.xmin, roi.ymax - roi.ymin)));

}

void BallDetector::createBlobImage(
    cv::Mat& mat, std::vector<BallBlob> goodBlobs,
    std::vector<BallBlob> badBlobs,
    std::string filepath) {

  cv::Mat seg = mat.clone();
  cv::cvtColor(seg, seg, CV_GRAY2BGR);
  seg.setTo(cv::Scalar(255, 255, 255));
  
  cv::Vec3b colorBlack(0,0,0);
  cv::Vec3b colorBlue(235,204,91);
  cv::Vec3b colorRed(0,0,255);
  cv::Vec3b colorPink(164,74,255);

  for(auto& blob : goodBlobs) {
    if (blob.wasMerged()) {
      colorBlob(seg, blob, colorBlue);
    } else {
      colorBlob(seg, blob, colorBlack);
    }
  }
  for(auto& blob : badBlobs) {
    if (blob.wasMerged()) {
      colorBlob(seg, blob, colorPink);
    } else {
      colorBlob(seg, blob, colorRed);
    }
  }

  std::cout << "saving image to " << filepath << std::endl;
  cv::imwrite(filepath, seg);
}

bool BallDetector::setBest(BallCandidate& candidate) {
  candidate.best = true;
  best_ = std::make_unique<BallCandidate>(candidate);
  auto& ball = getball();
  // if (candidate.confidence > 0) {
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
    tlog(34,"selected ball at %3.0f,%3.0f dist=%2.2f, bear=%2.2f, elev=%2.2f",
      candidate.absPosition.x, candidate.absPosition.y,
      ball.visionDistance, ball.visionBearing * RAD_T_DEG, ball.visionElevation * RAD_T_DEG
    );

    // This whole block is a hack to skip localization
    if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH) {
      Point2D relBall(candidate.relPosition.x, candidate.relPosition.y);
      auto& self = getself();
      Point2D globalBall = relBall.relativeToGlobal(self.loc, self.orientation);
      ball.loc = globalBall;
      ball.bearing = ball.visionBearing;
      ball.elevation = ball.visionElevation;
      ball.relPos = relBall;
      ball.sd.x = 500;
      ball.sd.y = 500;
    }

    return true;
  // }
  // return false;
}

float BallDetector::getExpectedBlobCameraWidth(BallBlob& blob) {
  float worldHeight = 10.0;
  float worldWidth = 3.0;
  return cmatrix_.getExpectedCameraWidth((int)(blob.centroidX), (int)(blob.centroidY), worldHeight, worldWidth);
}

float BallDetector::getDirectDistanceByBlobWidth(float dx, int /*imageX*/, int /*imageY*/) {
//  dx *= 640.0f / iparams_.width; // Normalize because this was tuned with a width of 640
  //float dist = 24269.169211 * powf(dx,-0.904299);  // tuned for exploreUT13
//  float dist = 27615.38886 * powf(dx,-0.9571350451); // tuned for US Open 14 by katie
  float dist = cmatrix_.getWorldDistanceByWidth(dx, BALL_RADIUS*2);
  return dist;
}

float BallDetector::getDirectDistanceByKinematics(int x, int y) {
  Position p = cmatrix_.getWorldPosition(x, y, BALL_RADIUS);
  float dist = cmatrix_.directDistance(p);
  return dist;
}

float BallDetector::getDirectDistance(BallCandidate& candidate) {
  float dist;
  float wdist = getDirectDistanceByBlobWidth(2.0f*candidate.radius, candidate.centerX, candidate.centerY);
  float kdist = getDirectDistanceByKinematics(candidate.centerX, candidate.centerY);

  // Kdist is better up close, wdist is better farther away. We scale the ratio of each so that we
  // don't get large discrepancies at the cutoff points
  float minKdist = 1000, maxKdist = 3000;
  float wdistRatio = pow((kdist - minKdist) / (maxKdist - minKdist), 2);
  if(kdist > maxKdist) dist = wdist;
  else if (kdist > minKdist) dist = kdist * (1 - wdistRatio) + wdist * wdistRatio;
  else dist = kdist;

  candidate.kwDistanceDiscrepancy = fabs(kdist - wdist) / (kdist + wdist);

  tlog(34,"ball candidate (%i) width dist: %3.0f  kin dist: %3.0f  wdistRatio: %0.3f",candidate.index,wdist,kdist,wdistRatio);
  return dist;
}
