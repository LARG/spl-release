#include <vision/HoughRobotDetector.h>
#include <iomanip>


#ifdef USER_sanmit

#ifdef TOOL
  #define DEBUG_OUTPUT false
#else
  #define DEBUG_OUTPUT false
#endif
#else
  #define DEBUG_OUTPUT false
#endif

using namespace Eigen;
using namespace cv;

HoughRobotDetector::HoughRobotDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter, BlobDetector& blob_detector, HoughDetector& hough_detector) :
  DETECTOR_INITIALIZE, color_segmenter_(segmenter), blob_detector_(blob_detector), hough_detector_(hough_detector) { 
  estimator_.setMean(1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.4f, 0.3f, 1.0f, ROBOT_CHEST_HEIGHT);  
  estimator_.setStdDev(
   0.5f, // w/h
   0.75f, // h/w
   0.7f, // kdist/wdist
   0.7f, // kdist/hdist
   0.3f, // torso/feet discrepancy
   0.6f, // jersey %
   0.3f, // green white %
   0.3f, // white %
   0.6f,  // correct %
   100.0f // chest height
  );

  
  color_segmenter_.getStepSize(hstep, vstep);  
  
  
  
}






void HoughRobotDetector::detectRobots(){

  vector<Blob> vertLines = hough_detector_.vertLines;


  // Expand all the blobs till they hit green
  RobotCandidate *expandedBlobs = new RobotCandidate[vertLines.size() - 1];
  for (int i = 0; i < vertLines.size()-1; i++){
    int ymax = expandRobot(vertLines[i], vertLines[i+1]);
    expandedBlobs[i].xi = vertLines[i].xi;
    expandedBlobs[i].xf = vertLines[i+1].xf;
    expandedBlobs[i].yi = min(vertLines[i].yi, vertLines[i+1].yi);
    expandedBlobs[i].yf = ymax;
    expandedBlobs[i].used = false;
  }


  // Merge and form robot candidates

  vector<RobotCandidate> robotCandidates;


  // Loop over all blobs
  for (int i = 0; i < vertLines.size()-1; i++){

    if (expandedBlobs[i].used)
      continue;


    expandedBlobs[i].used = true;
    RobotCandidate candidate;
    candidate.xi = expandedBlobs[i].xi;
    candidate.xf = expandedBlobs[i].xf;
    candidate.yi = expandedBlobs[i].yi;
    candidate.yf = expandedBlobs[i].yf;
    candidate.numLines = 1;

    Position posi = cmatrix_.getWorldPosition(expandedBlobs[i].xi, expandedBlobs[i].yf);

    // Try merging with successive blobs (up to a ythresh) as long as they are on the same ground plane location
    for (int j = i+1; j < vertLines.size(); j++){
      
      Position posj = cmatrix_.getWorldPosition(expandedBlobs[j].xf, expandedBlobs[j].yf);

      // Check if merging with this would make too wide
      if (abs(posi.y - posj.y) > 500){
        //if (DEBUG_OUTPUT) printf("Breaking search on match for blob %d\n", i);
        break;
      }
     
      // Check if on the same ground plane
      if (abs(posi.x - posj.x) > 300){
        
        //if (DEBUG_OUTPUT) printf("Skipping blob %d since diffDist away is %f\n", j, abs(posi.x -posj.x));
        continue;

      }
      

      // Merge
      expandedBlobs[j].used = true;
      candidate.numLines++;
      candidate.xf = expandedBlobs[j].xf;
      candidate.yf = max(expandedBlobs[j].yf, candidate.yf);



    }

    if (candidate.numLines > 1){
      
      
      // Verify candidate is on the field
      
      
      
      
      
      robotCandidates.push_back(candidate);
    
      if (DEBUG_OUTPUT) printf("Robot Candidate %lu: width %f numLines %d\n", robotCandidates.size()-1, abs(posi.y - cmatrix_.getWorldPosition(candidate.xf, candidate.yf).y), candidate.numLines);
    
    
    }

  }


  
  
/*

  vector<Blob> robotCandidates;

  int xMergeThresh = 0;   //5


  for (int i = 0; i < vertLines.size()-1; i++){


    int ymax = expandRobot(vertLines[i], vertLines[i+1]);

    if (i > 0 && vertLines[i].xi - robotCandidates.back().xf <= xMergeThresh ){
      
      // Verify y overlap
//      if (!(line[3] - postCandidates.back().yf > yMergeThresh || postCandidates.back().yi - line[1] > yMergeThresh)){
      
        Blob *lastBlob = &robotCandidates.back();
        lastBlob->xf = vertLines[i].xf;
        if (vertLines[i].yi < lastBlob->yi){
          lastBlob->yi = vertLines[i].yi;
        }
        if (ymax > lastBlob->yf){
          lastBlob->yf = ymax;
        }
        lastBlob->edgeSize += vertLines[i].edgeSize;
        lastBlob->edgeStrength += vertLines[i].edgeStrength;
    }
    // Push back and create new blob
    else {
      Blob blob;
      blob.xi = vertLines[i].xi;
      blob.yi = vertLines[i].yi;
      blob.xf = vertLines[i].xf;
      blob.yf = ymax;
      blob.edgeSize = vertLines[i].edgeSize;
      blob.edgeStrength = vertLines[i].edgeStrength;
      robotCandidates.push_back(blob);
    }



  }
*/


#ifdef TOOL
  goalPostImage = color::rawToMat(vblocks_.image->getImgTop(), iparams_);
  Scalar blue(255, 0, 0);
  Scalar green(0, 255, 0);
  Scalar red(0, 0, 255);
  Scalar colors[3] = {blue, green, red};



  for (int i = 0; i < robotCandidates.size(); i++){

    RobotCandidate *candidate = &robotCandidates[i];
    rectangle(goalPostImage, cv::Point(candidate->xi, candidate->yi), cv::Point(candidate->xf, candidate->yf), Scalar(0,0,0), 1, CV_AA);

  }

  imwrite("/home/sanmit/Desktop/robots.png", goalPostImage);

#endif



  


  printf("\n");


}





// Return the image y coordinate of the base of the post, after expansion
int HoughRobotDetector::expandRobot(const Blob &bi, const Blob &bj){
      
  int eYMin = min(bi.yf, bj.yf);
  int eXMin = bi.xf;
  int eXMax = bj.xi;
  int eYMax;
  eXMin = std::max(eXMin, 0) - (eXMin % hstep);
  eXMax = std::min(eXMax, iparams_.width - 1) - (eXMax % hstep);
  eYMin = std::max(eYMin, 0) - (eYMin % vstep);

   
  for ( eYMax = eYMin; eYMax <= (iparams_.height - 1); eYMax+=vstep){

    int gRowCount = 0;
    int wRowCount = 0;
    int rowCount = 0;

    for (int x = eXMin; x<= eXMax; x+= hstep){
      
      rowCount++;
      Color c = color_segmenter_.xy2color(x,eYMax);
      if (c == c_WHITE || c == c_ROBOT_WHITE){
        wRowCount++;
      }
      else if (c == c_FIELD_GREEN){
        gRowCount++;
      }

    }

    // Stop expanding the post if ...
    float wFrac = 1.0 * wRowCount / rowCount;
    float gFrac = 1.0 * gRowCount / rowCount;
    
    if (gFrac > 0.4){
      break;
    }

  }
  
  return max(eYMin, eYMax-vstep); 
}

























void HoughRobotDetector::detectBlueRobots() {
  blueRobots_.clear();
  detectRobots(c_BLUE);
}

void HoughRobotDetector::detectPinkRobots() {
  pinkRobots_.clear();
  detectRobots(c_PINK);
}

void HoughRobotDetector::detectRobots(Color c) {
  blob_detector_.formBlobs(c);
  blob_detector_.calculateBlobData(c);
  uint16_t mergeIndex[MAX_LINE_BLOBS];
  blob_detector_.mergeHorizontalBlobs(c, mergeIndex, MAX_LINE_BLOBS);
  std::vector<Blob>& blobs = blob_detector_.horizontalBlob[c];
  std::vector<Blob*> merged;
  for(unsigned int i = 0; i < blobs.size(); i++) {
    Blob& blob = blobs[i];
    if(mergeIndex[i] == MAX_LINE_BLOBS)
      merged.push_back(&blob);
  }
  std::vector<RobotCandidate> candidates = formRobotCandidates(merged, c);
  computeCandidateProbabilities(candidates);
  selectRobots(candidates);
}

std::vector<RobotCandidate> HoughRobotDetector::formRobotCandidates(std::vector<Blob*>& blobs, Color c) {
  std::vector<RobotCandidate> candidates;
  for(unsigned int i = 0; i < blobs.size(); i++) {
    Blob* blob = blobs[i];
    RobotCandidate candidate;
    candidate.blob = blob;
    candidate.width = blob->xf - blob->xi + 1;
    candidate.height = blob->yf - blob->yi + 1;
    candidate.centerX = (blob->xi + blob->xf) / 2.0f;
    candidate.centerY = (blob->yi + blob->yf) / 2.0f;
    candidate.color = c;
    fillFeet(candidate);
    candidate.relTorso = cmatrix_.getWorldPosition(candidate.centerX, candidate.centerY, ROBOT_CHEST_HEIGHT);
    candidate.relPosition = (candidate.relFeet + Position(candidate.relTorso.x, candidate.relTorso.y, 0)) / 2;
    candidate.widthDistance = getDistanceByWidth(candidate.width);
    candidate.heightDistance = getDistanceByHeight(candidate.height);
    candidate.kinematicsDistance = cmatrix_.groundDistance(candidate.relPosition);
    //printf("wdist: %2.2f, hdist: %2.2f, kdist: %2.2f\n", candidate.widthDistance, candidate.heightDistance, candidate.kinematicsDistance);
    candidate.worldHeight = cmatrix_.getWorldHeight(Coordinates(candidate.centerX, candidate.centerY), Coordinates(candidate.feetX, candidate.feetY));
    fillColorPercents(candidate);
    candidates.push_back(candidate);
  }
  return candidates;
}

void HoughRobotDetector::computeCandidateProbabilities(std::vector<RobotCandidate>& candidates) {
  for(unsigned int i = 0; i < candidates.size(); i++) {
    RobotCandidate& candidate = candidates[i];
    float widthHeightRatio = std::min(candidate.width / candidate.height, 1.0f);
    float heightWidthRatio = std::min(candidate.height / candidate.width, 1.0f);
    float kDistOverWDist = candidate.kinematicsDistance / candidate.widthDistance;
    float kDistOverHDist = candidate.kinematicsDistance / candidate.heightDistance;
    float torsoDist = cmatrix_.groundDistance(candidate.relTorso);
    float feetDist = torsoDist;
    if(!candidate.feetMissing) feetDist = cmatrix_.groundDistance(candidate.relFeet);
    float torsoFeetDistDiscrepancy = abs(feetDist - torsoDist) / (feetDist + torsoDist);
    //printf("feet: %2.2f, torso: %2.2f, disc: %2.2f\n", feetDist, torsoDist, torsoFeetDistDiscrepancy);
    float prob = estimator_.getLikelihood(
      widthHeightRatio,
      heightWidthRatio,
      kDistOverWDist,
      kDistOverHDist,
      torsoFeetDistDiscrepancy,
      candidate.jerseyColorPercent,
      candidate.greenWhitePercent,
      candidate.whitePercent,
      candidate.correctPercent,
      candidate.worldHeight
    );
    //estimator_.printLast();
    candidate.confidence = prob;
  }
}

void HoughRobotDetector::selectRobots(std::vector<RobotCandidate>& candidates) {
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
    wo->imageCenterX = candidate.centerX;
    wo->imageCenterY = candidate.centerY;
    wo->fromTopCamera = (camera_ == Camera::TOP);
    wo->seen = true;
    if(candidate.color == c_BLUE)
      blueRobots_.push_back(candidate.blob);
    else
      pinkRobots_.push_back(candidate.blob);
  }
}

std::list<Blob*> HoughRobotDetector::getBlueRobots() {
  return blueRobots_;
}

std::list<Blob*> HoughRobotDetector::getPinkRobots() {
  return pinkRobots_;
}

float HoughRobotDetector::getDistanceByWidth(float width) {
  return cmatrix_.getWorldDistanceByWidth(width, JERSEY_WIDTH);
}

float HoughRobotDetector::getDistanceByHeight(float height) {
  return cmatrix_.getWorldDistanceByHeight(height, JERSEY_HEIGHT);
}

void HoughRobotDetector::fillFeet(RobotCandidate& candidate) {
  int hstep, vstep;
  color_segmenter_.getStepSize(hstep, vstep);
  int vstart = ROUND(std::min((int)(candidate.centerY + candidate.height), iparams_.height - 1), vstep);
  int vend = ROUND(std::min(vstart + (int)candidate.height * 6, iparams_.height - 1), vstep);
  int hstart = ROUND(std::min((int)(candidate.centerX - candidate.width / 2), iparams_.width - 1), hstep);
  int hend = ROUND(std::min((int)(candidate.centerX + candidate.width / 2), iparams_.width - 1), hstep);
  int feetX = candidate.centerX;
  int feetY = iparams_.height - vstep;
  for(int y = vstart; y <= vend; y += vstep) {
    int green = 0, other = 0;
    for(int x = hstart; x <= hend; x += hstep) {
      Color c = color_segmenter_.xy2color(x, y);
      if(c == c_FIELD_GREEN) green++;
      else other++;
    }
    float greenPct = (float)green / (other + green);
    if(greenPct > .4) {
      feetY = y; break;
    }
  }
  if(feetY == iparams_.height - vstep) candidate.feetMissing = true;
  Position p = cmatrix_.getWorldPosition(feetX, feetY);
  candidate.feetY = feetY;
  candidate.feetX = candidate.centerX;
  candidate.relFeet = p;
}

void HoughRobotDetector::fillColorPercents(RobotCandidate& candidate) {
  int hstep, vstep;
  color_segmenter_.getStepSize(hstep, vstep);
  int correct = 0, green = 0, white = 0, rwhite = 0, total = 0;
  for(int y = candidate.blob->yi; y <= candidate.blob->yf; y += vstep) {
    for(int x = candidate.blob->xi; x <= candidate.blob->xf; x += hstep) {
      Color c = color_segmenter_.xy2color(x, y);
      if(c == candidate.color) correct++;
      else if(c == c_WHITE) white++;
      else if(c == c_ROBOT_WHITE) rwhite++;
      else if(c == c_FIELD_GREEN) green++;
      total++;
    }
  }
  candidate.jerseyColorPercent = (float)correct / total;
  for(int y = candidate.blob->yf + vstep; y <= candidate.feetY; y += vstep) {
    for(int x = candidate.blob->xi; x <= candidate.blob->xf; x += hstep) {
      Color c = color_segmenter_.xy2color(x, y);
      if(c == candidate.color) correct++;
      else if(c == c_WHITE) white++;
      else if(c == c_ROBOT_WHITE) rwhite++;
      else if(c == c_FIELD_GREEN) green++;
      total++;
    }
  }

  candidate.greenWhitePercent = (float)(white + rwhite + green) / total;
  candidate.whitePercent = (float)(white + rwhite) / total;
  candidate.correctPercent = (float)(correct + white + rwhite + green) / total;
}

