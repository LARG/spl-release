#include "CrossDetector.h"
#define getself() (&vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF])

CrossDetector::CrossDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter, BlobDetector& blob_detector) :
  DETECTOR_INITIALIZE, color_segmenter_(segmenter), blob_detector_(blob_detector) {
  estimator_.setMean(
    1.0f, 1.0f, 1.0f,
    1.0f,      1.0f,
    1.0f, 1.0f, 1.0f,
    0.0f,
    PENALTY_MARK_SIZE, PENALTY_MARK_SIZE
  );
  estimator_.setStdDev(
    0.2f, 0.2f, 0.2f,
    0.2f,      0.2f,
    0.2f, 0.2f, 0.2f,
    1000.0f,
    100.0f, 100.0f
  );
}

void CrossDetector::detectCrosses() {
  

  VisionTimer::Start(47, "CrossDetector(%s)::blobs", camera_);
  
  int mergeCount = 100;
  uint16_t mergeIndex[mergeCount];
  BlobCollection blobsUnmerged, blobs;
  blob_detector_.formBlobs(blobs, c_WHITE);
  blob_detector_.calculateBlobData(blobs, c_WHITE);
  int mergeAttempts = 1;
  for(int i = 0; i < mergeAttempts; i++) {
    blobsUnmerged = blobs; blobs.clear();
    blob_detector_.mergeBlobs(blobsUnmerged, mergeIndex, mergeCount);
    for(unsigned int i = 0; i < blobsUnmerged.size(); i++) {
      Blob& blob = blobsUnmerged[i];
      if(mergeIndex[i] == mergeCount) {
        blobs.push_back(blob);
      }
    }
  }
  VisionTimer::Stop("CrossDetector(%s)::blobs", camera_);

  
  VisionTimer::Start(47, "CrossDetector(%s)::all_crosses", camera_);
  // TODO: Sanmit: Cross offset points to our PK cross. Does this mean it bypasses localization??? Need to look into it. Also it's unlikely we'd ever see 2 crosses in the same frame. 
  // for(int i = 0; i < NUM_CROSSES; i++) {
  //   WorldObject* cross = &vblocks_.world_object->objects_[i + CROSS_OFFSET];
  detectCross(blobs);  // , cross);
  // }


  VisionTimer::Stop("CrossDetector(%s)::all_crosses", camera_);

}

void CrossDetector::detectCross(BlobCollection& blobs) {  // , WorldObject* cross) {
  int hstep, vstep;
  color_segmenter_.getStepSize(hstep, vstep);

  
  Pose2D self(getself()->orientation, getself()->loc.x, getself()->loc.y); // release
  //Pose2D self(0, 250 - HALF_FIELD_X, 0); // Standard goalie position
  for(uint16_t i = 0; i < blobs.size(); i++) {


    // VisionTimer::Start(47, "CrossDetector(%s)::cross_blob", camera_);

    Blob& blob = blobs[i];
    int left = blob.xi, right = blob.xf, top = blob.yf, bottom = blob.yi;
    int width = right - left, height = top - bottom;
    int wideXRange = std::max(100, width), narrowXRange = std::max(width / 2, 50);
    int wideYRange = std::max(80, height), narrowYRange = std::max(height / 2, 30);
    //printf("x: [%i,%i,%i,%i], y: [%i,%i,%i,%i]\n", 
        //left - wideXRange, left - narrowXRange, right + narrowXRange, right + wideXRange,
        //bottom - wideYRange, bottom - narrowYRange, top + narrowYRange, top + wideYRange
        //);

    // Throw out if we found a ball in the same camera here. This isn't a perfect solution...
    WorldObject ball = vblocks_.world_object->objects_[WO_BALL];
    if (ball.seen && ((ball.fromTopCamera && camera_ == Camera::TOP) || (!ball.fromTopCamera && camera_ == Camera::BOTTOM))){
      int ballX = ball.imageCenterX;
      int ballY = ball.imageCenterY;
      int ballR = ball.radius;
      int padding = 8;
      // Check if cross blob center is inside ball
      if (pow(blob.avgX - ballX, 2) + pow(blob.avgY - ballY, 2) <= pow(ball.radius + padding, 2)){
        tlog(47, "Skipping blob %i (%i,%i to %i,%i) since too close to ball", i, blob.xi, blob.yi, blob.xf, blob.yf);
        continue;  
      }
    }

    // Another check to eliminate balls: check the gray/black(i.e. c_BLUE) in the lower half
    float grayBelow = checkGrayBelow(left, right, bottom + (height / 2), top + (height / 2), hstep, vstep);
    if (grayBelow > 0.350){
        tlog(47, "Skipping blob %i (%i,%i to %i,%i) since too much gray around %f", i, blob.xi, blob.yi, blob.xf, blob.yf, grayBelow);
      continue;
    }


    // VisionTimer::Stop("CrossDetector(%s)::cross_blob", camera_);

    // VisionTimer::Start(47, "CrossDetector(%s)::cross_pos", camera_);
    Position 
      lpos = cmatrix_.getWorldPosition(left, (top + bottom) / 2),
      rpos = cmatrix_.getWorldPosition(right, (top + bottom) / 2),
      tpos = cmatrix_.getWorldPosition((right + left) / 2, top),
      bpos = cmatrix_.getWorldPosition((right + left) / 2, bottom),
      relCenter = cmatrix_.getWorldPosition((right + left) / 2, (top + bottom) / 2);

    Pose2D absCenter = Pose2D(relCenter.x, relCenter.y).relativeToGlobal(self);


    // VisionTimer::Stop("CrossDetector(%s)::cross_pos", camera_);



    // VisionTimer::Start(47, "CrossDetector(%s)::cross_gpct", camera_);

    float wWidth = (lpos - rpos).abs(), wHeight = (tpos - bpos).abs();
    WorldObject* cross1 = &vblocks_.world_object->objects_[CROSS_OFFSET];
    WorldObject* cross2 = &vblocks_.world_object->objects_[1 + CROSS_OFFSET];
    float pDist = std::min( (absCenter.translation - Vector2<float>(cross1->loc.x, cross1->loc.y)).abs(),
                            (absCenter.translation - Vector2<float>(cross2->loc.x, cross2->loc.y)).abs());
    /* float pDist = 1000000;
    for(int i = 0; i < NUM_CROSSES; i++) {
      WorldObject* cross = &vblocks_.world_object->objects_[i + CROSS_OFFSET];
      pDist = std::min(pDist, (absCenter.translation - Vector2<float>(cross->loc.x, cross->loc.y)).abs());
    } */
    if(vblocks_.game_state->state() != PLAYING) pDist = 0; // This shouldn't be used when not in playing, helps with debugging

    float gtopleft = getGreenPercentage(left - wideXRange, left - narrowXRange, top + narrowYRange, top + wideYRange, hstep, vstep);
    float gtopmid = getGreenPercentage(left - narrowXRange, right + narrowXRange, top + narrowYRange, top + wideYRange, hstep, vstep);
    float gtopright = getGreenPercentage(right + narrowXRange, right + wideXRange, top + narrowYRange, top + wideYRange, hstep, vstep);

    float gleft = getGreenPercentage(left - wideXRange, left - narrowXRange, bottom - narrowYRange, top + narrowYRange, hstep, vstep);
    float gright = getGreenPercentage(right + narrowXRange, right + wideXRange, bottom - narrowYRange, top + narrowYRange, hstep, vstep);

    float gbottomleft = getGreenPercentage(left - wideXRange, left - narrowXRange, bottom - wideYRange, bottom - narrowYRange, hstep, vstep);
    float gbottommid = getGreenPercentage(left - narrowXRange, right + narrowXRange, bottom - wideYRange, bottom - narrowYRange, hstep, vstep);
    float gbottomright = getGreenPercentage(right + narrowXRange, right + wideXRange, bottom - wideYRange, bottom - narrowYRange, hstep, vstep);


    // VisionTimer::Stop("CrossDetector(%s)::cross_gpct", camera_);


    tlog(47, "Blob %i (%i,%i to %i,%i) chosen for cross detection", i, blob.xi, blob.yi, blob.xf, blob.yf);
    tlog(47, "Self at %2.f,%2.f, candidate at %2.f,%2.f", self.translation.x, self.translation.y, absCenter.translation.x, absCenter.translation.y);

    // VisionTimer::Start(47, "CrossDetector(%s)::cross_likelihood", camera_);

    float prob = estimator_.getLikelihood(
     gtopleft,
     gtopmid,
     gtopright,
     gleft,
     gright,
     gbottomleft,
     gbottommid,
     gbottomright,
     pDist,
     wWidth,          // World width
     wHeight          // World height
    );
    //estimator_.printLast();
    estimator_.logLast(47, textlogger);
    // VisionTimer::Stop("CrossDetector(%s)::cross_likelihood", camera_);

    if(prob > .4) {
      tlog(47, "Blob %i selected as cross", i);
      WorldObject* ucross = &vblocks_.world_object->objects_[WO_UNKNOWN_PENALTY_CROSS];
      setCrossObject(blob, ucross);
    }
  }
}

void CrossDetector::correctRanges(int& xmin, int& xmax, int& ymin, int& ymax, int hstep, int vstep) {
  xmin = std::max(0, xmin);
  xmax = std::min(iparams_.width - 1, xmax);
  ymin = std::max(0, ymin);
  ymax = std::min(iparams_.height - 1, ymax);
  xmin = xmin - (xmin % hstep);
  ymin = ymin - (ymin % vstep);
}


// Check whether there is gray in the lower half and below the candidate (this is to throw out candidates on the ball)
// Another option was to add ROBOT_WHITE in the green below check, but this cuts out some good crosses.
// Also doing this separately because I don't know exactly what regions that function is checking
float CrossDetector::checkGrayBelow(int xmin, int xmax, int ymin, int ymax, int hstep, int vstep){

  tlog(47, "  checking gray below");
  tlog(47, "Scanning from x:%i->%i, y:%i->%i", xmin, xmax, ymin, ymax);
  correctRanges(xmin,xmax,ymin,ymax,hstep,vstep);
  int numTotal = 0, numGray = 0;
  for(int x = xmin; x <= xmax; x += hstep) {
    for(int y = ymin; y <= ymax; y += vstep) {
      Color c = color_segmenter_.xy2color(x,y);
      numTotal++;
      if(c == c_ROBOT_WHITE || c == c_BLUE) numGray++;
    }
  }
  if (numTotal == 0) return 0.0;
  tlog(47, "  ball pct: %f", (float)numGray/numTotal);
  return (float) numGray / numTotal;
}



float CrossDetector::getGreenPercentage(int xmin, int xmax, int ymin, int ymax, int hstep, int vstep) {
  //printf("Scanning from x:%i->%i, y:%i->%i\n", xmin, xmax, ymin, ymax);
  tlog(47, "Scanning from x:%i->%i, y:%i->%i", xmin, xmax, ymin, ymax);
  correctRanges(xmin,xmax,ymin,ymax,hstep,vstep);
  int numTotal = 0, numGreen = 0;
  for(int x = xmin; x <= xmax; x += hstep) {
    for(int y = ymin; y <= ymax; y += vstep) {
      Color c = color_segmenter_.xy2color(x,y);
      numTotal++;
      if(c == c_FIELD_GREEN) numGreen++;
      if(c == c_WHITE || c == c_ROBOT_WHITE || c == c_BLUE) return 0.0; // There shouldn't be any white or ball colored pixels around the cross
    }
  }
  if(numTotal == 0) return 0.0; // No false positives, no regrets
  tlog(47, "  green pct: %f", (float)numGreen/numTotal);
  return (float)numGreen / numTotal;
}

float CrossDetector::getDistanceByBlobWidth(float dx) {
  dx *= 640.0f / iparams_.width; // Tuned with a width of 640
  return .9f * 16765.0f * powf(dx, -0.769f);
}

void CrossDetector::setCrossObject(Blob& blob, WorldObject* cross) {
  int left = blob.xi, right = blob.xf, yval = blob.yi;
  int centerX = left + (right - left) / 2;
  Position p = cmatrix_.getWorldPosition(centerX, yval);
  cross->visionDistance = cmatrix_.groundDistance(p);
  cross->visionBearing = cmatrix_.bearing(p);
  cross->visionElevation = cmatrix_.elevation(p);
  cross->seen = true;
  cross->fromTopCamera = true;
  cross->imageCenterX = centerX;
  cross->imageCenterY = yval;
  cross->visionConfidence = 1.0;
  cross->frameLastSeen = vblocks_.frame_info->frame_id;
  tlog(47, "Cross identified at %i, %i", centerX, yval);
}
