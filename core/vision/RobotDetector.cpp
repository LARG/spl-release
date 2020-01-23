#include <vision/RobotDetector.h>
#include <iomanip>

using namespace Eigen;

#define getself() vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF]

RobotDetector::RobotDetector(DETECTOR_DECLARE_ARGS, FieldEdgeDetector& field_edge_detector, ColorSegmenter& segmenter) :
  DETECTOR_INITIALIZE, field_edge_detector_(field_edge_detector), color_segmenter_(segmenter) { 
  
    
    
    
    
  estimator_.setMean(1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.4f, 0.3f, 1.0f, ROBOT_CHEST_HEIGHT);  
  estimator_.setStdDev(
   0.5f, // w/h
   0.75f, // h/w
   0.7f, // kdist/wdist
   0.7f, // kdist/hdist
   0.3f, // torso/feet discrepancy
   0.6f, //  %
   0.3f, // green white %
   0.3f, // white %
   0.6f,  // correct %
   100.0f // chest height
  );
}


void RobotDetector::detectRobots(){
  float GREEN_CHECK_THRESH = 0.5;


  if (camera_ == Camera::BOTTOM) return;

  // If comms are off, we don't want to do this since we use comm to distinguish between friends and foes
  if (vblocks_.robot_state->ignore_comms_) return;


  std::vector<RobotCandidate> candidates;


  // Extend robot objects to hull point boundaries

  for (int o = 0; o < field_edge_detector_.objects.size(); o++){


    VisionObjectCandidate &object = field_edge_detector_.objects[o];
    if (!object.valid || !object.robotCandidate || object.postCandidate) continue;

    
    // Find adjacent hull point and extend to there
    int start = object.xi;
    int end = object.xf;

    for (int i = 0; i < field_edge_detector_.hullPointCands.size(); i++){

      FieldEdgePoint &p = field_edge_detector_.hullPointCands[i];

//      printf("Point %f %f\n", p.x, p.y);

      if (!p.valid || p.below) continue;

      if (p.x > object.xf){
        end = p.x;
      }

      if (p.x <= object.xi){
        start = p.x;
        break;
      }

    }

    object.xi = start;
    object.xf = end;
   
    object.dx = object.xf - object.xi + 1;
    object.avgX = (object.xi + object.xf) / 2.0;
    
    // TODO: Let's not update the position yet since if there are 2 adjacent robots, it will get messed up.
    //    object.pos = cmatrix_.getWorldPosition(object.avgX, object.yf);

  }

  std::sort(field_edge_detector_.objects.begin(), field_edge_detector_.objects.end(), VisionObjectCandidate::sortByXi);

  // Remove duplicates
  for (int o = 0; o < field_edge_detector_.objects.size(); o++){
    
    VisionObjectCandidate &object = field_edge_detector_.objects[o];
    if (!object.valid || !object.robotCandidate || object.postCandidate) continue;


    for (int m = o+1; m < field_edge_detector_.objects.size(); m++){
      
      VisionObjectCandidate &object2 = field_edge_detector_.objects[m];
      if (!object2.valid || !object2.robotCandidate || object2.postCandidate) continue;

      // One of the objects is a duplicate. 
      if (object.xi == object2.xi && object.xf == object2.xf){


        if (object.yf < object2.yf){
          object.valid = false;
        }
        else {
          object2.valid = false;
        }
        break;
      }

      // If the x's are far apart, break
      if (object2.xi >= object.xf){
        break;
      }

    }

  }



  for (int o = 0; o < field_edge_detector_.objects.size(); o++){

    VisionObjectCandidate &object = field_edge_detector_.objects[o];



    if (!object.valid || !object.robotCandidate || object.postCandidate) continue;

	  int hstep, vstep;
	  color_segmenter_.getStepSize(hstep, vstep);
	  int green = 0, total=0;

	  int gXMin, gXMax, gYMin, gYMax;

	  gXMin = object.xi - (object.xi % hstep);
	  gXMax = object.xf - (object.xf % hstep);
	  
	  gYMin = object.yi - (object.yi % vstep);
	  gYMax = object.yf - (object.yf % vstep);               // If we don't take the whole image, then there will be white below. 



	  for(int y = gYMin; y <= gYMax; y += vstep) {
	    for(int x = gXMin; x <= gXMax; x += hstep) {
	      Color c = color_segmenter_.xy2color(x, y);
	      if(c == c_FIELD_GREEN) green++;
	      total++;
	    }
	  }
	  float green_prc = 1.0*green / total;

//	  printf("filtering green in candidate %d %d %d %d %f %d", object.xi, object.xf, object.yi, object.yf, green_prc, green_prc > GREEN_CHECK_THRESH);

	  if (green_prc > GREEN_CHECK_THRESH) {
	  	object.valid = false;
	  	continue;
	  }

      RobotCandidate candidate;

      candidate.xi = object.xi;
      candidate.xf = object.xf;
      candidate.yi = object.yi;
      candidate.yf = object.yf;

      candidate.avgX = object.avgX;
      candidate.avgY = object.avgY;

      candidate.width = object.dx;
      candidate.height = object.dy;
//      candidate.centerX = (blob->xi + blob->xf) / 2.0f;
//      candidate.centerY = (blob->yi + blob->yf) / 2.0f;
      
//      fillFeet(candidate);
      
//      candidate.relTorso = cmatrix_.getWorldPosition(candidate.centerX, candidate.centerY, ROBOT_CHEST_HEIGHT);
      
      candidate.relPosition = cmatrix_.getWorldPosition(candidate.avgX, candidate.yf);
      
      Point2D relPos(candidate.relPosition.x, candidate.relPosition.y);
      Point2D globPos = relPos.relativeToGlobal(getself().loc, getself().orientation);
      candidate.absPosition = Position(globPos.x, globPos.y, 0); 

      candidate.confidence = 1.0;

       // (candidate.relFeet + Position(candidate.relTorso.x, candidate.relTorso.y, 0)) / 2;
      
//      candidate.widthDistance = getDistanceByWidth(candidate.width);
//      candidate.heightDistance = getDistanceByHeight(candidate.height);
//      candidate.kinematicsDistance = cmatrix_.groundDistance(candidate.relPosition);
//      candidate.worldHeight = cmatrix_.getWorldHeight(Coordinates(candidate.centerX, candidate.centerY), Coordinates(candidate.feetX, candidate.feetY));
//      fillColorPercents(candidate);
      candidates.push_back(candidate);
     

  }


  const int TEAMMATE_DIST_THRESH = 1000;

  for (int i = WO_TEAM_FIRST; i <= WO_TEAM_LAST; i++){

    if (i == vblocks_.robot_state->WO_SELF) continue;

    // Check that this teammate is still active. I.e. check that it is not penalized
    RelayStruct *tp = vblocks_.team_packets->getPktPtr(i);
    if (tp->bvrData.state == PENALISED && vblocks_.game_state->state() != SET) continue;

    // TODO: what if you haven't heard from the robot in a long time?    


    // We can either use the WO info which is 1 frame old, or the team packets, which are presumably fresh? I guess it depends on the order in which they are processed?
    
    WorldObject *wo = &vblocks_.world_object->objects_[i];
//    printf("TEAMMATE %d: (%f,%f)\n", i - WO_TEAM_FIRST + 1, wo->loc.x, wo->loc.y);

    
    for (int c = 0; c < candidates.size(); c++){

      float distance = Point2D(wo->loc.x, wo->loc.y).getDistanceTo(Point2D(candidates[c].absPosition.x, candidates[c].absPosition.y));


      if (distance < TEAMMATE_DIST_THRESH){
        candidates[c].confidence = 0.0;
//        printf("Teammate %d found!\n", i);
      }

    }



  }
//  printf("------------------------\n");





  selectRobots(candidates);

}



void RobotDetector::selectRobots(std::vector<RobotCandidate>& candidates) {
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

//    printf("Detected opponent. Adding WO %d\n",  i + oppOffset);

    wo->visionDistance = cmatrix_.groundDistance(candidate.relPosition);
    wo->visionBearing = cmatrix_.bearing(candidate.relPosition);
    wo->visionElevation = cmatrix_.elevation(candidate.relPosition);
    wo->imageCenterX = candidate.avgX;
    wo->imageCenterY = candidate.yi;
    wo->fromTopCamera = (camera_ == Camera::TOP);
    wo->seen = true;
//    if(candidate.color == c_BLUE)
//      blueRobots_.push_back(candidate.blob);
//    else
//      pinkRobots_.push_back(candidate.blob);

  }
}




/// ==============

std::list<Blob*> RobotDetector::getBlueRobots() {
  return blueRobots_;
}

std::list<Blob*> RobotDetector::getPinkRobots() {
  return pinkRobots_;
}



// ======================================= //
// OLD JERSEY BASED DETECTION CODE
//
/*
void RobotDetector::detectBlueRobots() {
  blueRobots_.clear();
  detectRobots(c_BLUE);
}

void RobotDetector::detectPinkRobots() {
  pinkRobots_.clear();
  detectRobots(c_PINK);
}

void RobotDetector::detectRobots(Color c) {
  blob_detector_->formBlobs(c);
  blob_detector_->calculateBlobData(c);
  uint16_t mergeIndex[MAX_LINE_BLOBS];
  blob_detector_->mergeHorizontalBlobs(c, mergeIndex, MAX_LINE_BLOBS);
  std::vector<Blob>& blobs = blob_detector_->horizontalBlob[c];
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

std::vector<RobotCandidate> RobotDetector::formRobotCandidates(std::vector<Blob*>& blobs, Color c) {
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

void RobotDetector::computeCandidateProbabilities(std::vector<RobotCandidate>& candidates) {
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
      candidate.ColorPercent,
      candidate.greenWhitePercent,
      candidate.whitePercent,
      candidate.correctPercent,
      candidate.worldHeight
    );
    //estimator_.printLast();
    candidate.confidence = prob;
  }
}




float RobotDetector::getDistanceByWidth(float width) {
//  return cmatrix_.getWorldDistanceByWidth(width, _WIDTH);
  return 0;
}

float RobotDetector::getDistanceByHeight(float height) {
//  return cmatrix_.getWorldDistanceByHeight(height, _HEIGHT);
  return 0;
}

void RobotDetector::fillFeet(RobotCandidate& candidate) {
  int hstep, vstep;
  color_segmenter_->getStepSize(hstep, vstep);
  int vstart = ROUND(std::min((int)(candidate.centerY + candidate.height), iparams_.height - 1), vstep);
  int vend = ROUND(std::min(vstart + (int)candidate.height * 6, iparams_.height - 1), vstep);
  int hstart = ROUND(std::min((int)(candidate.centerX - candidate.width / 2), iparams_.width - 1), hstep);
  int hend = ROUND(std::min((int)(candidate.centerX + candidate.width / 2), iparams_.width - 1), hstep);
  int feetX = candidate.centerX;
  int feetY = iparams_.height - vstep;
  for(int y = vstart; y <= vend; y += vstep) {
    int green = 0, other = 0;
    for(int x = hstart; x <= hend; x += hstep) {
      Color c = color_segmenter_->xy2color(x, y);
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

void RobotDetector::fillColorPercents(RobotCandidate& candidate) {
  int hstep, vstep;
  color_segmenter_->getStepSize(hstep, vstep);
  int correct = 0, green = 0, white = 0, rwhite = 0, total = 0;
  for(int y = candidate.blob->yi; y <= candidate.blob->yf; y += vstep) {
    for(int x = candidate.blob->xi; x <= candidate.blob->xf; x += hstep) {
      Color c = color_segmenter_->xy2color(x, y);
      if(c == candidate.color) correct++;
      else if(c == c_WHITE) white++;
      else if(c == c_ROBOT_WHITE) rwhite++;
      else if(c == c_FIELD_GREEN) green++;
      total++;
    }
  }
  candidate.ColorPercent = (float)correct / total;
  for(int y = candidate.blob->yf + vstep; y <= candidate.feetY; y += vstep) {
    for(int x = candidate.blob->xi; x <= candidate.blob->xf; x += hstep) {
      Color c = color_segmenter_->xy2color(x, y);
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

void RobotDetector::detectRobotCluster() {
}

*/





