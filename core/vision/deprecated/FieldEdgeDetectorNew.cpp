#include "FieldEdgeDetectorNew.h"

using namespace cv;


/**

TODO:

 * Need to eliminate false edges by spurious intercepts
 * Eliminate edges too close to top of image (probably field just fills
 field of vision)
 * Add confidence to the line (RMS or consensus)
 * Identify potential robots
 */





FieldEdgeDetector::FieldEdgeDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) : DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) {

  hstep = (1 << iparams_.defaultHorizontalStepScale);
  vstep = (1 << iparams_.defaultVerticalStepScale);
  consensusCF = new int[iparams_.width / hstep];
  srand (time(NULL));
  VOTE_THRESH = 2 * vstep;


  timer1.setInterval(300);
  timer2.setInterval(300);
  timer3.setInterval(300);

  timer1.setMessage("== EDGE 1 ");
  timer2.setMessage("== EDGE 2 ");
  timer3.setMessage("== EDGE 3 ");

}

/** Main function that detects field edges. Detected edges will be put into fieldEdges array and populated into world objects */
bool FieldEdgeDetector::detectFieldEdges(){

  fieldEdgeCounter = 0;


#ifdef TOOL
  // Debugging images
  m_img = color::rawToMat(vblocks_.image->getImgTop(), iparams_);
  for (int x = 0; x < iparams_.width; x += hstep){
    circle(m_img, cv::Point(x, classifier_->fieldEdgePoints[x/hstep]), 2, Scalar(0, 200, 0), 2);
  }
#endif



  // Fit lines on left and right 
  Line2D left, right;
  bool foundLeft = ransacLine(0, (4* iparams_.width) /(10 * hstep), left);
  bool foundRight = ransacLine( (6 * iparams_.width) / (10 * hstep), (iparams_.width / hstep) - 1, right);



  if (foundLeft && foundRight) {    


    // Check if they are the same line
    if (isOneLine(right, left)){
      Line2D whole;
      bool foundWhole = ransacLine(0, (iparams_.width / hstep) - 1, whole);

      // Filter line if found whole
      if (foundWhole){

        LineSegment lineSeg = truncateLine(whole, 0, (iparams_.width / hstep) - 1);
        projectAndFillEdge(lineSeg); 


#ifdef TOOL          
        line(m_img, cv::Point(0, whole.getYGivenX(0)), cv::Point(iparams_.width - 1, whole.getYGivenX(iparams_.width - 1)), Scalar(0, 0, 0), 3);
        line(m_img, cv::Point(lineSeg.start.x, lineSeg.start.y), cv::Point(lineSeg.end.x, lineSeg.end.y), Scalar(0, 128, 255), 10);
#endif

      }
    } 
    else {
      Point2D intersection = left.getIntersection(right);
      LineSegment leftSeg(Point2D(0, left.getYGivenX(0)), intersection);
      LineSegment rightSeg(intersection, Point2D(iparams_.width-1, right.getYGivenX(iparams_.width - 1)));
      projectAndFillEdge(leftSeg);
      projectAndFillEdge(rightSeg);


#ifdef TOOL      
      line(m_img, cv::Point(0, left.getYGivenX(0)), cv::Point(intersection.x, left.getYGivenX(intersection.x)), Scalar(0, 255, 0), 2);
      line(m_img, cv::Point(intersection.x, right.getYGivenX(intersection.x)), cv::Point(iparams_.width - 1, right.getYGivenX(iparams_.width - 1)), Scalar(0, 0, 255), 2);
#endif

    }
  }

  else {

    if (foundLeft){

      LineSegment leftSeg = truncateLine(left, 0, (iparams_.width / hstep) - 1);
      projectAndFillEdge(leftSeg);

#ifdef TOOL
      line(m_img, cv::Point(0, left.getYGivenX(0)), cv::Point(iparams_.width - 1, left.getYGivenX(iparams_.width - 1)), Scalar(0, 255, 0), 2);
      line(m_img, cv::Point(leftSeg.start.x, leftSeg.start.y), cv::Point(leftSeg.end.x, leftSeg.end.y), Scalar(0, 128, 255), 10);
#endif

    }

    if (foundRight){

      LineSegment rightSeg = truncateLine(right, 0, (iparams_.width / hstep) - 1);
      projectAndFillEdge(rightSeg);

#ifdef TOOL      
      line(m_img, cv::Point(0, right.getYGivenX(0)), cv::Point(iparams_.width - 1, right.getYGivenX(iparams_.width - 1)), Scalar(0, 0, 255), 2);
      line(m_img, cv::Point(rightSeg.start.x, rightSeg.start.y), cv::Point(rightSeg.end.x, rightSeg.end.y), Scalar(0, 128, 255), 10);
#endif

    }

  }


#ifdef TOOL

#ifdef USER_sanmit  
  imwrite("/home/sanmit/Desktop/fieldEdge.png", m_img);
#endif

#ifdef USER_eladlieb
  imwrite("/home/eladlieb/Desktop/fieldEdge.png", m_img);
#endif

#endif  

}


/** Projects line segment from image space to world space, and populates fieldEdges array and WO_UNKNOWN_FIELD_EDGES */
bool FieldEdgeDetector::projectAndFillEdge(LineSegment &lineSeg){

  // Found too many lines or something weird happened. 
  if (fieldEdgeCounter >= 2 || fieldEdgeCounter < 0)
    return false;

  // Project to world space
  Position start = cmatrix_.getWorldPosition(lineSeg.start.x, lineSeg.start.y);
  Position end = cmatrix_.getWorldPosition(lineSeg.end.x, lineSeg.end.y);
  fieldEdges[fieldEdgeCounter].start = Point2D(start.x, start.y);
  fieldEdges[fieldEdgeCounter].end = Point2D(end.x, end.y);

  // Populate World Object
  //  setFieldEdgeObject(WO_UNKNOWN_FIELD_EDGE_1 + fieldEdgeCounter, fieldEdges[fieldEdgeCounter]);

  // Update field edge line counter
  fieldEdgeCounter++;

  return true;
}



LineSegment FieldEdgeDetector::truncateLine(Line2D &line, int start, int end){

  int bestStartIndex = 0;
  int bestLength = 0;

  const int WINDOW_SIZE = 30;
  const float CONSENSUS_THRESH = CONSENSUS_PCT * WINDOW_SIZE;

  int cur_s = -1;     // Current segment start
  int cur_len = 0;    // Current segment length


  // Dynamic programming find consensus set
  // Initialize
  if (line.getDistanceToPoint(Point2D(start*hstep, classifier_->fieldEdgePoints[start])) <= VOTE_THRESH){
    consensusCF[start] = 1;
  }
  else{
    consensusCF[start] = 0;
  }
  // Recurse
  for (int x = start+1; x <= end; x++){
    if (line.getDistanceToPoint(Point2D(x*hstep, classifier_->fieldEdgePoints[x])) <= VOTE_THRESH){
      consensusCF[x] = consensusCF[x-1] + 1;
    }
    else {
      consensusCF[x] = consensusCF[x-1];
    }
  }

  // Find longest consensus segment
  for (int i=start;i <= end-WINDOW_SIZE;i++){
    if (consensusCF[i+WINDOW_SIZE] - consensusCF[i] > CONSENSUS_THRESH) {
      // Start new segment
      if (cur_s < 0){
        cur_s = i;
        cur_len = WINDOW_SIZE;
      } 
      // Extend current segment
      else {
        cur_len++;
      }
    }
    else {
      // Terminating a segment, we check if it's longer than current best
      if (bestLength < cur_len){ 
        bestStartIndex = cur_s;
        bestLength = cur_len;
      }
      //Resetting segment
      cur_s = -1;
      cur_len = 0;
    }
  }
  // Check last segment
  if (bestLength < cur_len){ 
    bestStartIndex = cur_s;
    bestLength = cur_len;
  }

  //  printf("Best start index: %d length: %d\n", bestStartIndex*hstep, bestLength );

  return LineSegment(bestStartIndex * hstep, line.getYGivenX(bestStartIndex * hstep), (bestStartIndex+bestLength - 1) * hstep, line.getYGivenX((bestStartIndex + bestLength - 1) * hstep));

}


void FieldEdgeDetector::findPointsBelowLine(Line2D line){

  for (int x = 0; x < iparams_.width / hstep; x++){


    Position pointLoc = cmatrix_.getWorldPosition(x*hstep, classifier_->fieldEdgePoints[x]);
    Position lineLoc = cmatrix_.getWorldPosition(x*hstep, line.getYGivenX(x*hstep));


    if (sqrt(pow(pointLoc.x - lineLoc.x, 2) + pow(pointLoc.y - lineLoc.y, 2)) > 1000 && classifier_->fieldEdgePoints[x] > line.getYGivenX(x*hstep)){
      circle(m_img, cv::Point(x*hstep, classifier_->fieldEdgePoints[x]), 2, Scalar(0, 0, 255), 2);
    }

  }


}

/** Populates the given WorldObject (by index) given a line segment in world space */
void FieldEdgeDetector::setFieldEdgeObject(int woIndex, const LineSegment &edge){
  Point2D robot = Point2D(0, 0);
  Point2D closestPoint = edge.getPointOnLineClosestTo(robot);
  float distance = robot.getDistanceTo(closestPoint);
  float angle = robot.getAngleTo(closestPoint);
  vblocks_.world_object->objects_[woIndex].seen = true;
  vblocks_.world_object->objects_[woIndex].frameLastSeen = vblocks_.frame_info->frame_id;
  vblocks_.world_object->objects_[woIndex].visionDistance = distance;
  vblocks_.world_object->objects_[woIndex].visionBearing = angle;
  vblocks_.world_object->objects_[woIndex].visionElevation = 0;
  vblocks_.world_object->objects_[woIndex].visionPt1 = edge.start; 
  vblocks_.world_object->objects_[woIndex].visionPt2 = edge.end;
  vblocks_.world_object->objects_[woIndex].visionLine = edge;
  vblocks_.world_object->objects_[woIndex].fromTopCamera = (camera_ == Camera::TOP);
}


/** Tries to find a line using RANSAC using field edge point candidates in the specified range */
bool FieldEdgeDetector::ransacLine(int start, int end, Line2D &bestLine){

  const int MAX_ITERS = 10;
  const int test_skip = 2;
  const float CONSENSUS_THRESH = CONSENSUS_PCT * (end - start + 1)/test_skip;
  float best_consensus = -1;

  int pt1, pt2;
  for (int i = 0; i < MAX_ITERS; i++){

    timer1.start();

    int pt1X = (rand() % (end-start+1)) + start;
    int pt2X = (rand() % (end-start+1)) + start;
    /*printf("sampled x coordinates are %d,%d\n",pt2X*hstep,pt1X*hstep);*/

    timer1.stop();
    timer1.printAtInterval();

    if (pt1X == pt2X)
      continue;

    timer2.start();

    Line2D line = Line2D::makeLineFromTwoPoints(Point2D(pt1X*hstep, classifier_->fieldEdgePoints[pt1X]), Point2D(pt2X*hstep, classifier_->fieldEdgePoints[pt2X]));
    int consensus = 0;
    for (int p = start; p <= end; p+=test_skip){
      if (line.getDistanceToPoint(Point2D(p*hstep, classifier_->fieldEdgePoints[p])) <= VOTE_THRESH){
        consensus++;
      }
    }

    timer2.stop();
    timer2.printAtInterval();

    if (consensus > CONSENSUS_THRESH){
      bestLine.m_a = line.m_a;
      bestLine.m_b = line.m_b;
      bestLine.m_c = line.m_c;
      best_consensus = consensus;
      pt1 = pt1X;
      pt2 = pt2X;
      return true;
    }

  }

  return false;

  /*  
      if(best_consensus > CONSENSUS_THRESH) {

      timer3.start();

      Line2D lineObj = Line2D::makeLineFromTwoPoints(Point2D(pt1*hstep, classifier_->fieldEdgePoints[pt1]), Point2D(pt2*hstep, classifier_->fieldEdgePoints[pt2]));

      timer3.stop();
      timer3.printAtInterval();

#ifdef TOOL    
circle(m_img, cv::Point(pt1*hstep, classifier_->fieldEdgePoints[pt1]), 3, Scalar(255,0, 0), 3);
circle(m_img, cv::Point(pt2*hstep, classifier_->fieldEdgePoints[pt2]), 3, Scalar(255,0, 0), 3);

line(m_img, cv::Point(pt1*hstep, classifier_->fieldEdgePoints[pt1]), cv::Point(pt2*hstep, classifier_->fieldEdgePoints[pt2]), Scalar(255, 255, 255), 5);
#endif


return true;
} else {
return false;
}
   */

  }

/** Returns whether the two lines are the same (i.e. roughly parallel and therefore likely the same), by checking angles in image space (world space seemed to be too noisy) */
bool FieldEdgeDetector::isOneLine(Line2D& lineA, Line2D& lineB){  

  const float ANGLE_DEV_THRESH = 0.135;//in radians

  AngRad ang_is = lineA.getAngleToLine(lineB);

  bool test = ang_is <= ANGLE_DEV_THRESH;
  /*
     printf("****************************\n");
     printf("angle in pixel space is %f\n", ang_is);
     printf("test result is %d\n" ,test);
     printf("****************************\n");
   */

  return test;


  /*  
  //This stuff is here for when we were trying to run the test in the projected space

  float lineA_x1 = 50.0;
  Position lineA_1 = cmatrix_.getWorldPosition(lineA_x1,lineA.getYGivenX(lineA_x1));
  float lineB_x1 = 50.0;
  Position lineB_1 = cmatrix_.getWorldPosition(lineB_x1,lineB.getYGivenX(lineB_x1));

  float lineA_x2 = 1250.0;
  Position lineA_2 = cmatrix_.getWorldPosition(lineA_x2,lineA.getYGivenX(lineA_x2));
  float lineB_x2 = 1250.0;
  Position lineB_2 = cmatrix_.getWorldPosition(lineB_x2,lineB.getYGivenX(lineB_x2));



  Line2D lineObjA = Line2D::makeLineFromTwoPoints(Point2D(lineA_1.x, lineA_1.y), Point2D(lineA_2.x, lineA_2.y));
  Line2D lineObjB = Line2D::makeLineFromTwoPoints(Point2D(lineB_1.x, lineB_1.y), Point2D(lineB_2.x, lineB_2.y));

  AngRad ang = lineObjA.getAngleToLine(lineObjB);
  AngRad desirable = 1.57;
  AngRad diff = abs(desirable - ang);

  if (diff < ANGLE_DEV_THRESH){
  return true;
  }
  return false; 

   */

}






// DO NOT REMOVE and leave at bottom
// vim: expandtab:noai:sts=2:sw=2:ts=2
