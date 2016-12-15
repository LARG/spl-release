#include "BallDetector.h"
#include <vision/Logging.h>

using namespace Eigen;

#define horizontalBlob blob_detector_.horizontalBlob
#define verticalBlob blob_detector_.verticalBlob
#define getball() (&vblocks_.world_object->objects_[WO_BALL])
#define getself() (&vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF])
#define getframe() vblocks_.frame_info->frame_id

BallDetector::BallDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter, BlobDetector& blob_detector) : DETECTOR_INITIALIZE, color_segmenter_(segmenter), blob_detector_(blob_detector) {
  for(int i = 0; i < MAX_BALL_CANDS; i++)
    candidates_[i] = new BallCandidate();
  estimator_.setMean( 
    1.0f, // orange %
    1.0f, // green/white %
    0.0f, // circle deviation
    0.0f, // height
    0.0f, // kinematics / width based distance discrepancy
    0.0f, // distance from field
    0.0f  // velocity
  );
}

/**
 * Called by ImageProcessor (in processFrame) to detect the ball on the field
 * using the current camera frame as input.
 */
void BallDetector::findBall() {
  bestBallCandIndex = -1;
  
  WorldObject *ball = getball();
  // if ball was already detected by bottom camera, and we're in top cam, done
  if(ball->seen && !ball->fromTopCamera && camera_ == Camera::TOP) {
    tlog(34, "Ball was seen already in the bottom camera, bailing out.");
    return;
  }

  blob_detector_.resetOrangeBlobs();
#ifdef TOOL
  vblocks_.robot_vision->doHighResBallScan = true;
#endif
  ballCandCount = 0;

  if(color_segmenter_.startHighResBallScan()) {
    tlog(34, "Started high res scan for balls in the top camera");
    color_segmenter_.didHighResBallScan = true;
  }
  blob_detector_.formOrangeBlobs();
  blob_detector_.calculateOrangeBlobData();
  checkAndSelectBall();
  color_segmenter_.completeHighResScan();
  vblocks_.robot_vision->doHighResBallScan = false;
}

/**
 * Uses a Blob Detector to find the best ball candidate from orange blobs.
 * Best candidate is selected based on how well it matches the size and shape
 * of a ball (formBallCandidates() gets the ball data for each candidate, and
 * computeCandidateProbabilities() evaluates the confidence and validity).
 * @return True if a good ball candidate is seen, false if not.
 */
bool BallDetector::checkAndSelectBall()
{
  ballCandCount = 0;

  if (horizontalBlob[c_ORANGE].size() > MAX_ORANGE_BLOBS) {
    tlog(34,  "Too many orange blobs in the scene");
    return true;
  }

  if (horizontalBlob[c_ORANGE].empty()) {
    tlog(34,  "No orange blobs in the scene");
    return false;
  }

  // merge orange blobs
  uint16_t mergeIndex[MAX_ORANGE_BLOBS];
  blob_detector_.mergeHorizontalBlobs(c_ORANGE, mergeIndex, MAX_ORANGE_BLOBS);

  std::vector<Blob*> blobs;

  for(unsigned int i = 0; i < horizontalBlob[c_ORANGE].size(); i++) {
    Blob* blob = &horizontalBlob[c_ORANGE][i];
    if(mergeIndex[i] == MAX_ORANGE_BLOBS) {
      blobs.push_back(blob);
      tlog(34, "merged blob %i: (xi:xf,yi:yf) = (%i:%i,%i:%i)", i, blob->xi, blob->xf, blob->yi, blob->yf);
    }
  }

  // sort orange blobs by blob area size
  //blobs = sortOrangeBlobs(blobs);
  sort(blobs.begin(), blobs.end(), sortBlobAreaPredicate);

  for(unsigned int i = 0; i < blobs.size(); i++) {
    Blob* blob = blobs[i];
    tlog(34, "sorted blob %i: (xi:xf,yi:yf) = (%i:%i,%i:%i)", i, blob->xi, blob->xf, blob->yi, blob->yf);
  }

  // fit circles to ball candidates_
  tlog(34, "forming ball candidates");
  std::vector<BallCandidate*> candidates = formBallCandidates(blobs);
  BallCandidate *best;
  //best = sanityCheckBallCands(candidates);
  tlog(34, "computing candidate probabilities");
  best = computeCandidateProbabilities(candidates);
  if(!best) return false;
  setBestBallCandidate(best);
  return true;
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

/**
 * Uses data from the detected ball candidates to evaluate the probabilities of
 * each and determines the best ball candidate from the choices.
 * @param A vector of all detected ball candidates.
 * @return The BallCandidate pointer to the most likely true ball.
 */
BallCandidate* BallDetector::computeCandidateProbabilities(std::vector<BallCandidate*> candidates) {
  tlog(34, "processing %i candidates", candidates.size());
  if(candidates.size() == 0) return false;
  Position ballRelPos(getball()->relPos.x, getball()->relPos.y, 0);
  double ballDistance = cmatrix_.groundDistance(ballRelPos);
  bool ballInitialized = (getball()->frameLastSeen > 0);
  const float confThreshold = .3;
  float maxV = 100;
  float maxD = 100;
  // Scale max velocity with expected distance since positional uncertainty increases w/ range
  if(ballInitialized) {
    maxV = std::max(100.0f, ballDistance * .1f);
    maxD = std::max(100.0f, ballDistance * .2f);
  }
  estimator_.setStdDev(0.5f, 0.4f, 0.3f, 260.0f, 0.4f, maxD, maxV);
  for(uint16_t i = 0; i < candidates.size(); i++) {
    tlog(34, "processing candidate %i", i);
    BallCandidate* candidate = candidates[i];
    Blob* blob = candidate->blob;
    int bw = blob->xf - blob->xi, bh = blob->yf - blob->yi;
    if(
        // Look at the blobs for these since half circle fitting can alter the height/width calculations.
        bw < 2 ||
        bh < 2 ||
        candidate->blob->correctPixelRatio < .25 // These are all over the place so they don't really fit into the Gaussian model
    ) { 
      tlog(34, "threw out ball candidate because: xf - xi = %i (min 2), yf - yi = %i (min 2), pixel ratio %2.2f (min .25)", bw, bh, candidate->blob->correctPixelRatio);
      candidate->confidence = 0; continue; 
    }
    
    // The bottom camera can only see the robot's shoulder between 60 and 120 degrees on either side, so throw out all candidates here
    if(intersectsShoulder(candidate)) {
        tlog(34, "threw out ball candidate for being on robot shoulder");
        candidate->confidence = 0;
        continue;
    }

    float circleOrangePct = checkColorsInCircleFit(candidate);
    float belowGreenWhitePct = camera_ == Camera::TOP ? checkBelowGreenWhitePct(candidate) : 1.0;
    float circleFit = candidate->stddev;
    float height = candidate->relPosition.z;

    Point2D pa = Point2D(abs(candidate->absPosition.x), abs(candidate->absPosition.y));
    float xd = (pa.x > HALF_GRASS_X ? pa.x - HALF_GRASS_X : 0), yd = (pa.y > HALF_GRASS_Y ? pa.y - HALF_GRASS_Y : 0);
    //printf("pa: %2.2f,%2.2f, xd: %2.2f, yd: %2.2f\n", pa.x, pa.y, xd, yd);
    float distanceFromField = sqrt(xd * xd + yd * yd);

    Position diff = ballRelPos - candidate->relPosition;
    int dt = getframe() - getball()->frameLastSeen;
    double dx = abs(diff.x), dy = abs(diff.y);
    float v = sqrt(dx * dx + dy * dy) / dt;
#ifdef TOOL
    v = 0; // Velocity can be problematic when running logs, especially when log frames aren't sequential
#endif
    if(!ballInitialized || dt == 0) v = 0;
    if(vblocks_.game_state->state() == TESTING) distanceFromField = 0; // Don't turn this off for SET or PLAYING or the robots will literally explode - JM 06/01/15

    double prob = estimator_.getLikelihood(
      circleOrangePct,
      belowGreenWhitePct,
      circleFit,
      height,
      candidate->kwDistanceDiscrepancy,
      distanceFromField,
      v
    );
    //estimator_.printLast();
    tlog(34, "Checking candidate %i: %i,%i to %i,%i", i, blob->xi, blob->yi, blob->xf, blob->yf);
    estimator_.logLast(34,textlogger);

    candidate->confidence = prob;
  }
  BallCandidate* best = NULL; float bestConf = 0;
  for(uint16_t i = 0; i < candidates.size(); i++) {
    BallCandidate* candidate = candidates[i];
    if(candidate->confidence < confThreshold){
      tlog(34, "ball candidate %i thrown out for having confidence %f lower than threshold %f", i, candidate->confidence, confThreshold);
      continue;
    }
    // Final check to get rid of balls not on the field
    if (camera_ == Camera::TOP && checkGreenAround(candidate) < 0.25){
      tlog(34, "ball thrown out for not having enough green around");
      continue;
    }
    if(candidate->confidence > bestConf) {
      best = candidate;
      bestConf = candidate->confidence;
    }
  }
  if(!best) return NULL;
  //printf("best position: %2.2f,%2.2f\n", best->absPosition.x, best->absPosition.y);
  return best;
}

/** checks the percent of pink between a given set of points */
float BallDetector::checkPinkPct(int x1, int x2, int y1, int y2){
  if (x1 < 0) x1 = 0;
  if (y1 < 0) y1 = 0;
  if (x2 >= iparams_.width) x2 = iparams_.width-1;
  if (y2 >= iparams_.height) y2 = iparams_.height-1;

  int numTotal = 0;
  int numPink = 0;
  for (int x = x1; x <= x2; x+=4){
    for (int y = y1; y <= y2; y+=2){
      numTotal++;
      int c = getSegPixelValueAt(x, y);
      if (c == c_PINK) numPink++;
    }
  }

  //tlog(43, "check from x: %i, %i, and y: %i, %i, total: %i, pink: %i",
  //    x1, x2, y1, y2, numTotal, numPink);

  if (numTotal == 0)
    return 0.0;

  return (float)numPink/(float)numTotal;

}


/** Todd: this checks along 4 vectors from the ball for pink. */
bool BallDetector::checkSurroundingPink(int i){

  int roundX = 4;
  int roundY = 2;
  int pixToCheck = 10;

  int midY = ((int)(candidates_[i]->centerY/roundY))*roundY;
  int midX = ((int)(candidates_[i]->centerX/roundX))*roundX;

  int endX1 = ((int)((candidates_[i]->centerX - candidates_[i]->radius)/roundX))*roundX;
  int startX1 = endX1 - roundX * pixToCheck;

  float check1 = checkPinkPct(startX1, endX1, midY-roundY*2, midY+roundY*2);

  int startX2 = ((int)((candidates_[i]->centerX + candidates_[i]->radius)/roundX))*roundX;
  int endX2 = startX2 + roundX * pixToCheck;

  float check2 = checkPinkPct(startX2, endX2, midY-roundY*2, midY+roundY*2);

  int endY1 = ((int)((candidates_[i]->centerY - candidates_[i]->radius)/roundY))*roundY;
  int startY1 = endY1 - roundY * pixToCheck;

  float check3 = checkPinkPct(midX - roundX*2, midX+roundX*2, startY1, endY1);

  int startY2 = ((int)((candidates_[i]->centerY + candidates_[i]->radius)/roundY))*roundY;
  int endY2 = startY2 + roundY * pixToCheck;

  float check4 = checkPinkPct(midX - roundX*2, midX+roundX*2, startY2, endY2);

  tlog(43, "Nearby pink %f, %f, %f, %f", check1, check2, check3, check4);

  // Todd: for good balls, these never got over 0.09
  // for orange in pink bands, there was always one over 0.16
  // going to set threshold at 0.15
  if (check1 > 0.15 || check2 > 0.15 || check3 > 0.15 || check4 > 0.15){
    return false;
  }

  return true;

}


float BallDetector::checkGreenAround(BallCandidate *candidate) {


  int hstep = 1 << iparams_.defaultHorizontalStepScale, vstep = 1 << iparams_.defaultVerticalStepScale;

  // Open a region around the ball
  int ymin = candidate->blob->yi - (candidate->radius * 2);
  int ymax = candidate->blob->yf + (candidate->radius * 2);
  int xmin = candidate->blob->xi - (candidate->radius * 2);
  int xmax = candidate->blob->xf + (candidate->radius * 2);

  // Map to pixels that are classified, and within the image boundaries
  ymin -= ymin % vstep;
  ymax += vstep - ymax % vstep;
  xmin -= xmin % hstep;
  xmax += hstep - xmax % hstep;

  xmin = std::max(xmin, 0);
  xmax = std::min(xmax, iparams_.width - 1);
  ymin = std::max(ymin, 0);
  ymax = std::min(ymax, iparams_.height - 1);

  int total = 0, green = 0;
  for (int x = xmin; x <= xmax; x += hstep){
    for (int y = ymin; y <= ymax; y += vstep){
      int c = getSegPixelValueAt(x, y);
      if (c != c_ORANGE){
        total++;
        green += (c == c_FIELD_GREEN);
      }
    }
  }

  float pct = (float)green/total;
  if(!total && ymin >= iparams_.height)
  // We can't see below the ball due to the height, so we assume 1. There
  // is no reason the robot should see anything else that's orange and that
  // extends vertically below the lower boundary of the image.
    pct = 1;


  return pct;
}

/** Check the percantage of green/white below ball. */
float BallDetector::checkBelowGreenWhitePct(BallCandidate* candidate) {
  // We use the default step sizes because the area under the ball shouldn't be high-res scanned
  int hstep = 1 << iparams_.defaultHorizontalStepScale, vstep = 1 << iparams_.defaultVerticalStepScale;

  int ymin = candidate->blob->yf, ymax = candidate->blob->yf + candidate->radius * 2;
  ymax += vstep - ymax % vstep;
  int xmin = candidate->blob->xi, xmax = candidate->blob->xf;
  ymin -= ymin % vstep;
  ymax += vstep - ymax % vstep;
  xmin -= xmin % hstep;
  xmax += hstep - xmax % hstep;

  xmin = std::max(xmin, 0);
  xmax = std::min(xmax, iparams_.width - 1);
  ymin = std::max(ymin, 0);
  ymax = std::min(ymax, iparams_.height - 1);

  int green = 0, white = 0, total = 0;
  int orange = 0, black = 0;
  for (int x = xmin; x <= xmax; x += hstep){
    for (int y = ymin; y <= ymax; y += vstep){
      int c = getSegPixelValueAt(x, y);
      total++;
      green += (c == c_FIELD_GREEN);
      white += (c == c_WHITE || c == c_ROBOT_WHITE);
      orange += (c == c_ORANGE);
      black += (c == c_UNDEFINED);
    }
  }

  float pct = (float)(green + white) / total;

  //printf("h:%i,v:%i,x:[%i,%i], y:[%i,%i], green: %i, white: %i, orange: %i, black: %i, total: %i, pct: %2.2f\n",
      //hstep, vstep, xmin, xmax, ymin, ymax, green, white, orange, black, total, pct);

  if(!total && ymin >= iparams_.height)
  // We can't see below the ball due to the height, so we assume 1. There
  // is no reason the robot should see anything else that's orange and that
  // extends vertically below the lower boundary of the image.
    pct = 1;

  tlog(43, "check from x: %i, %i, and y: %i, %i, hstep: %i, vstep: %i, total: %i, greenwhite: %i, pct: %5.3f orange %i black %i", xmin, xmax, ymin, ymax, hstep, vstep, total, green + white, pct, orange, black);

  return pct;
}


/** This checks the percentage of orange inside in the fitted ball circle
 *  (or, reality, inside the biggest square that can fit inside that circle).
 *  It helps prevent false balls when we fit orange to a small sliver of orange.
 * @param The ball candidate.
 * @return The percentage of orange in the candidate.
 */
float BallDetector::checkColorsInCircleFit(BallCandidate* candidate){
  // check pixels inside circle radius
  // whats the length of one side of the largest square inside the circle
  float squareWidth = sqrtf((4.0*candidate->radius*candidate->radius)/2.0);

  int hstep, vstep;
  color_segmenter_.getStepSize(hstep,vstep);
  int roundX = hstep;
  int roundY = vstep;

  int startX = ((int)((candidate->centerX - squareWidth/2.0)/roundX))*roundX;
  int startY = ((int)((candidate->centerY - squareWidth/2.0)/roundY))*roundY;
  int endX = candidate->centerX + squareWidth/2.0;
  int endY = candidate->centerY + squareWidth/2.0;
  if (startX < 0) startX = 0;
  if (startY < 0) startY = 0;
  if (endX >= iparams_.width) endX = iparams_.width-1;
  if (endY >= iparams_.height) endY = iparams_.height-1;

  //tlog(43, "ball radius scan from x: %i, %i, and y: %i, %i, radius %f, sqWidth %f",
  //         startX, endX, startY, endY, candidate->radius, squareWidth);

  int totalPixelsChecked = 0;
  int numOrange = 0;
  //int numUndefined = 0;
  //int numPink = 0;
  for (int x = startX; x < endX; x+=hstep){
    for (int y = startY; y < endY; y+=vstep){
      totalPixelsChecked++;
      int c = getSegPixelValueAt(x, y);
      if (c == c_ORANGE) numOrange++;
      // if (c == c_PINK) numPink++;
      //if (c == c_UNDEFINED) numUndefined++;
    }
  }

  //tlog(43, "ball radius scan total %i, orange %i, undef %i, pink %i",
  //          totalPixelsChecked, numOrange, numUndefined, numPink);
  if (totalPixelsChecked == 0) {
    return 0;
  }

  float pctOrange = (float)numOrange / (float)totalPixelsChecked;
  //tlog(43, "pctOrange in circle fit %f", pctOrange);

  return pctOrange;
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
 * Uses the detected blob data to form a candidate using the parameters of the
 * detected ball. This function fills out information about the balls size,
 * distance, on-screen position, etc.
 * @return A BallCandidate pointer (potentially representing the true ball).
 * @param The blob to be used to form this candidate, and the index of the
 *        candidate.
 */
BallCandidate* BallDetector::formBallCandidate(Blob* blob, int ballCandIndex) {
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
  BallCandidate* candidate = candidates_[ballCandIndex];
  //std::cout << "fit points 1: (cx,cy), r, sd: " << "(" << cxF << "," << cyF << "), " << rF << ", " << sdF << "\n";
  if (2 * bl->dy > 3 * bl->dx) {
    float cxL, cyL, rL, sdL;                                       // left
    fitCircleToPoints(contLX, contLY, contLCount, &cxL, &cyL, &rL, &sdL);
    float cxR, cyR, rR, sdR;                                       // rite
    fitCircleToPoints(contRX, contRY, contRCount, &cxR, &cyR, &rR, &sdR);

    // set final ball candidate circle fit values
    //const uint16_t cxBlob = (bl->xi + bl->xf) >> 1;
    if (cxR > bl->xf) {
      tlog(15,  "<<< LEFT contour chosen !! >>>");
      candidate->centerX = cxL;
      candidate->centerY = cyL;
      candidate->radius = rL;
      candidate->width = 2 * rL;
      candidate->stddev = sdL;
    } else if (cxL < bl->xi) {
      tlog(15,   "<<< RITE contour chosen !! >>>");
      candidate->centerX = cxR;
      candidate->centerY = cyR;
      candidate->radius = rR;
      candidate->width = 2 * rR;
      candidate->stddev = sdR;
    }
  } else {
    tlog(15,  "<<< FULL contour chosen: (cx,cy) = (%f,%f), rad = %f, stddev = %f !! >>>", cxF, cyF, rF, sdF);
    candidate->centerX = cxF;
    candidate->centerY = cyF;
    candidate->radius = rF;
    candidate->stddev = sdF;
    candidate->width = blob->dx;
  }
  candidate->index = ballCandIndex;
  candidate->blob = blob;
  candidate->height = blob->dy;
  float directDistance = getDirectDistance(candidate);
  candidate->relPosition = cmatrix_.getWorldPositionByDirectDistance(candidate->centerX, candidate->centerY, directDistance);
  candidate->groundDistance = cmatrix_.groundDistance(candidate->relPosition);
  //printf("dd: %2.2f, rel pos: %2.2f,%2.2f,%2.2f, gd: %2.2f\n", 
      //directDistance, candidate->relPosition.x, candidate->relPosition.y, candidate->relPosition.z, candidate->groundDistance);
  candidate->relPosition.z -= BALL_RADIUS;
  candidate->valid = true;
  Pose2D self(getself()->orientation, getself()->loc.x, getself()->loc.y); // release
  //Pose2D self(M_PI / 2, 0, -3000); // midfield
  //Pose2D self(M_PI / 2, 0, 0); // center facing right
  //Pose2D self(3 * M_PI / 4, 4000, -3000); // corner w/ near goal to the right
  Pose2D cand = Pose2D(candidate->relPosition.x, candidate->relPosition.y).relativeToGlobal(self);
  candidate->absPosition = Position(cand.translation.x, cand.translation.y, candidate->relPosition.z);
  return candidate;
}

/**
 * Forms ball candidates with probabilities and ball data from blobs.
 * @return A vector of BallCandidate objects evaluated from blobs.
 */
std::vector<BallCandidate*> BallDetector::formBallCandidates(std::vector<Blob*> blobs) {
  std::vector<BallCandidate*> candidates;
  for(uint16_t i = 0; i < blobs.size() && i < MAX_BALL_CANDS; i++) {
    BallCandidate* candidate = formBallCandidate(blobs[i], i);
    candidates.push_back(candidate);
  }
  return candidates;
}

/**
 * Fills in the ball (World Object) with the information from the best ball
 * candidate.
 * @param The best ball candidate.
 * @return True if the candidate confidence was higher than 0, false otherwise.
 */
bool BallDetector::setBestBallCandidate(BallCandidate* candidate){
  WorldObject *ball = getball();
  if (candidate->confidence > 0) {
    ball->reset();
    ball->radius = candidate->radius;
    ball->imageCenterX = candidate->centerX;
    ball->imageCenterY = candidate->centerY;
    ball->visionDistance = candidate->groundDistance;
    ball->visionBearing = cmatrix_.bearing(candidate->relPosition);
    ball->visionElevation = cmatrix_.elevation(candidate->relPosition);
    ball->seen = true;
    ball->visionConfidence = candidate->confidence;
    ball->frameLastSeen = getframe();
    ball->fromTopCamera = (camera_ == Camera::TOP);
    bestBallCandIndex = candidate->index;
    tlog(34,"selected ball at %3.0f,%3.0f dist=%2.2f, bear=%2.2f, elev=%2.2f",
        candidate->absPosition.x, candidate->absPosition.y,
        ball->visionDistance, ball->visionBearing * RAD_T_DEG, ball->visionElevation * RAD_T_DEG
        );

    // This whole block is a hack to skip localization
    if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH) {
      Point2D relBall(candidate->relPosition.x, candidate->relPosition.y);
      WorldObject *self = getself();
      Point2D globalBall = relBall.relativeToGlobal(self->loc, self->orientation);
      ball->loc = globalBall;
      ball->bearing = ball->visionBearing;
      ball->elevation = ball->visionElevation;
      ball->relPos = relBall;
      ball->sd.x = 500;
      ball->sd.y = 500;
    }

    return true;
  }
  return false;
}

float BallDetector::getDirectDistanceByBlobWidth(float dx, int /*imageX*/, int /*imageY*/) {
  dx *= 640.0f / iparams_.width; // Normalize because this was tuned with a width of 640
  //printf("dx: %0.3f", dx);
  //float dist = 24269.169211 * powf(dx,-0.904299);  // tuned for exploreUT13
  float dist = 27615.38886 * powf(dx,-0.9571350451); // tuned for US Open 14 by katie
  //printf("\tBall distance for diameter(%0.3f): %0.3f\n",dx,dist);
  return dist;
}

float BallDetector::getDirectDistanceByKinematics(int x, int y) {
  Position p = cmatrix_.getWorldPosition(x, y, BALL_RADIUS);
  float dist = cmatrix_.directDistance(p);
  return dist;
}

float BallDetector::getDirectDistance(BallCandidate* candidate) {
  float dist;
  float wdist = getDirectDistanceByBlobWidth(2.0f*candidate->radius, candidate->centerX, candidate->centerY);
  float kdist = getDirectDistanceByKinematics(candidate->centerX, candidate->centerY);

  // Kdist is better up close, wdist is better farther away. We scale the ratio of each so that we
  // don't get large discrepancies at the cutoff points
  float minKdist = 1000, maxKdist = 3000;
  float wdistRatio = pow((kdist - minKdist) / (maxKdist - minKdist), 2);
  if(kdist > maxKdist) dist = wdist;
  else if (kdist > minKdist) dist = kdist * (1 - wdistRatio) + wdist * wdistRatio;
  else dist = kdist;

  candidate->kwDistanceDiscrepancy = fabs(kdist - wdist) / (kdist + wdist);

  tlog(34,"ball candidate (%i) width dist: %3.0f  kin dist: %3.0f  wdistRatio: %0.3f",candidate->index,wdist,kdist,wdistRatio);
  return dist;
}

/**
 * @return True if a best candidate was found, false otherwise.
 */
bool BallDetector::bestCandidateFound() {
    return (bestBallCandIndex != ((uint16_t)-1));
}

void BallDetector::setHorizon(HorizonLine horizon) {
    horizon_ = horizon;
}

// DO NOT REMOVE and leave at bottom
// vim: expandtab:noai:sts=2:sw=2:ts=2
