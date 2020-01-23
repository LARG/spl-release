#include <vision/GoalDetector.h>
// #include <vision/ml/DeepClassifier.h>
#include <vision/ml/SvmClassifier.h>
#include <VisionCore.h>
#include <string>
#define ENABLE_CALIBRATION false
using namespace cv;

int GoalDetector::totalPosts = 0;

GoalDetector::GoalDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& color_segmenter, BlobDetector& blob_detector, LineDetector& line_detector, HoughDetector& hough_detector, FieldEdgeDetector& field_edge_detector) : 
  DETECTOR_INITIALIZE, color_segmenter_(color_segmenter), blob_detector_(blob_detector), line_detector_(line_detector), hough_detector_(hough_detector), field_edge_detector_(field_edge_detector), color(c_YELLOW) {
    // White goals code
  color_segmenter_.getStepSize(hstep, vstep); 
  totalValidGoals = 0;

  
  
  H << 1.0,  -0.1905, 0,
    0, 0.7143, 0,
    0, -0.0003, 1.0000;


  
//  H << 1.0,  -0.0952, 0,
//    0, 0.7143, 0,
//    0, -0.0012, 1.0000;

  Hinv << 1, 0.1333, 0,
       0, 1.4, 0,
       0, 0.0017, 1;
  
}

GoalDetector::~GoalDetector() {
}

void GoalDetector::init(TextLogger* tl) {
  printf("Initializing %s goal detector.\n", Camera::c_str(camera_));
  ObjectDetector::init(tl);
  auto path = VisionCore::inst_->memory_->data_path_ + "/models";
  printf("Loading deepnet goal detector files for %s...", Camera::c_str(camera_)); fflush(stdout);
  classifier_ = std::make_unique<SvmClassifier>("goal", path);
  classifier_->load();
  printf("done!\n");
}

// Return the image y coordinate of the base of the post, after expansion
int GoalDetector::expandPost(const Blob *bi, const Blob *bj){
  // Expand post lines based on green/white percentage below
  int eYMin = min(bi->yf, bj->yf);
  int eXMin = bi->xf;
  int eXMax = bj->xi;
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
    if (wFrac < 0.5){
      break;
    }
  }
  return max(eYMin, eYMax-vstep); 
}

float GoalDetector::calculatePostWhitePercentage(LineSegment leftEdge, LineSegment rightEdge, int xstep, int ystep){

  int postArea = 0;
  int wCounter = 0;

//  printf("Left (%.0f,%.0f) (%.0f,%.0f)\nRight (%.0f,%.0f) (%.0f,%.0f)\n", leftEdge.start.x, leftEdge.start.y, leftEdge.end.x, leftEdge.end.y, rightEdge.start.x, rightEdge.start.y, rightEdge.end.x, rightEdge.end.y);


  int ymin = leftEdge.start.y;
  int ymax = leftEdge.end.y;
  ymin = std::max(ymin, 0) - (ymin % ystep);
  ymax = std::min(ymax, iparams_.height -1) - (ymax % ystep);

//  printf("Checking white y from %d to %d\n", ymin, ymax);
  for (int y = ymin; y <= ymax; y+= ystep){
  

//    printf("%d, x from %.0f to %.0f\n", y, leftEdge.getXGivenY(y), rightEdge.getXGivenY(y));

    int xmin = static_cast<int>(leftEdge.getXGivenY(y));
    int xmax = static_cast<int>(rightEdge.getXGivenY(y));
    xmin = std::max(xmin, 0) - (xmin % xstep);
    xmax = std::min(xmax, iparams_.width -1) - (xmax % xstep);

//    printf("%d, x from %d to %d\n", y, xmin, xmax);

    for (int x = xmin; x <= xmax; x+= xstep){
      postArea++;
      Color c = color_segmenter_.xy2color(x, y);
      if (c == c_WHITE || c == c_ROBOT_WHITE){
        wCounter++;
      }     
    }

  }
  float whitePercentage = (float)wCounter / (float)postArea;
//  if (!postArea && pYMin >= iparams_.height)
//    whitePercentage = 1;
  if (!postArea)
    whitePercentage = 0;

//  printf("Found %f white\n\n", whitePercentage);
  return whitePercentage;

}

float GoalDetector::calculatePostWhitePercentage(int xmin, int xmax, int min_y, int max_y) {

  int postArea = 0;
  int wCounter = 0;
  int pXMin = xmin; //bi->xf;   // bi->xi
  int pXMax = xmax; //bj->xi;   // bj->xf
  int pYMin = min_y;
  int pYMax = max_y;
  pXMin = std::max(pXMin, 0) - (pXMin % hstep);
  pXMax = std::min(pXMax, iparams_.width - 1) - (pXMax % hstep);
  pYMin = std::max(pYMin, 0) - (pYMin % vstep);
  pYMax = std::min(pYMax, iparams_.height - 1) - (pYMax % vstep);
  for (int y = pYMin; y <= pYMax; y+=vstep){
    int greenInRowCount = 0;
    for (int x = pXMin; x <= pXMax; x+=hstep){
      postArea++;
      Color c = color_segmenter_.xy2color(x, y);
      if (c == c_WHITE || c == c_ROBOT_WHITE){
        wCounter++;
      }
      else if(c == c_FIELD_GREEN){
        greenInRowCount++;
      }
    }
  }
  float whitePercentage = (float)wCounter / (float)postArea;
  if (!postArea && pYMin >= iparams_.height)
    whitePercentage = 1;
  return whitePercentage;
}


float GoalDetector::calculateGreenPercentBelow(int gXMin, int gXMax, int min_y, int max_y){

  // Green percentage below, if possible. TODO: This assumes we detected the whole post...
  int greenCount = 0;
  int totalBelow = 0;
  // Adapt hstep/vstep?
  int height = max_y - min_y;
  int gYMin = max_y + (0.02 * height);
  int gYMax = max_y + (0.2 * height); 
  //      int gXMin = bi->xi;
  //      int gXMax = bj->xf;
  // Convert to coordinates where we have classified pixels
  gXMin = std::max(gXMin, 0) - (gXMin % hstep);
  gXMax = std::min(gXMax, iparams_.width - 1) - (gXMax % hstep);
  gYMin = std::max(gYMin, 0) - (gYMin % vstep);
  gYMax = std::min(gYMax, iparams_.height - 1) - (gYMax % vstep);               // If we don't take the whole image, then there will be white below. 
  for (int x = gXMin; x <= gXMax; x+= hstep){
    for (int y = gYMin; y <= gYMax; y+= vstep){
      totalBelow++;
      Color c = color_segmenter_.xy2color(x,y);
      if (c == c_FIELD_GREEN){
        greenCount++; 
      }
    }
  }
  float greenPct = (float)greenCount / (float)totalBelow;
  if (!totalBelow && gYMin >= iparams_.height)
    greenPct = 1;     // This heuristic was inherited from old code. 

  return greenPct;
}

void GoalDetector::formGoalPostCandidates(const vector<Blob> &vertLines){

  tlog(50, "Forming goal post candidates from hough lines");
//  const double WHITE_PCT_THRESH = 0.7;
//  const int MAX_POST_WIDTH_THRESH = 225;
//  const int MIN_POST_WIDTH_THRESH = 30;
//  const double GREEN_PCT_THRESH = 0.4;
//  tlog(50, "Threshold requirements: min_dist %d max_dist %d whiteFrac %f greenFrac %f", MIN_POST_WIDTH_THRESH, MAX_POST_WIDTH_THRESH, WHITE_PCT_THRESH, GREEN_PCT_THRESH);
  
  for (int i = 0; i < (int)vertLines.size()-1; i++){
    const Blob *bi = &vertLines[i];


    // SANMIT: instead of checking all the next lines, we should just check the next 1-3. If there are too many lines separating them, its probably not a post. 
    // So, changed j to range up to i+1+3 instead of vertLines.size()
    // Note that this isn't perfect. If it starts finding line blobs underneath the posts, it could screw up. Maybe a better criteria would be useful. 
    for (unsigned int j = i+1; j < i+1+1 && j < vertLines.size(); j++){

      const Blob *bj = &vertLines[j];


      // Calculate the base of the post
      int max_y = expandPost(bi, bj);
      int min_y = min(bi->yi, bj->yi);      // min
      //      int max_y = max(bi->yf, bj->yf);

      Position posi = cmatrix_.getWorldPosition(bi->xf, max_y);
      Position posj = cmatrix_.getWorldPosition(bj->xi, max_y);

      //      Position posi = cmatrix_.getWorldPosition(bi->xf, bi->yf);
      //      Position posj = cmatrix_.getWorldPosition(bj->xf, bj->yf);
      //      float dist = cmatrix_.getWorldDistanceByWidth(bj->avgX - bi->avgX, GOAL_POST_WIDTH);

      float whitePercentage = calculatePostWhitePercentage(bi->xf, bj->xi, min_y, max_y);
      float greenPct = calculateGreenPercentBelow(bi->xi, bj->xf, min_y, max_y);
      float euclidDist = sqrt(pow(posi.x - posj.x, 2) + pow(posi.y -posj.y, 2));

      float postWidth = abs(posi.y - posj.y);
      // abs(posi.y - posj.y) < 225

      float edgeRatio = (1.0 * bi->edgeStrength / bi->edgeSize) / (1.0 * bj->edgeStrength / bj->edgeSize);

      bool edgeRatioCheck = true; //(edgeRatio > 0.83) && (edgeRatio < 1.2);

      std::string matchFound = ""; 
      // White 0.8, posDiff 225, green 0.4, min post width 30
      // After china: min postWidth 50
      if (whitePercentage > 0.7 && postWidth < 225 && postWidth > 30 && greenPct > 0.4 && edgeRatioCheck){  
        GoalPostCandidate candidate;
        candidate.xi = bi->xi; //bi->avgX; //bi->xi;
        candidate.xf = bj->xf; //bj->avgX; //bj->xf;
        candidate.yi = min_y;
        candidate.yf = max_y;
        candidate.avgX = (candidate.xi + candidate.xf) / 2;
        candidate.avgY = (candidate.yi + candidate.yf) / 2;
        candidate.invalid = false;
        candidate.invalidIndex = -1;
        candidate.edgeSize = bi->edgeSize + bj->edgeSize;
        candidate.edgeStrength = bi->edgeStrength + bj->edgeStrength;
        candidate.leftEdgeWidth = bi->xf - bi->xi + 1;
        candidate.rightEdgeWidth = bj->xf - bj->xi + 1;
        candidate.width = postWidth;
        candidate.greenBelowPct = greenPct;
        candidate.whitePct = whitePercentage;
        candidate.relPosition = cmatrix_.getWorldPosition(candidate.avgX, candidate.yf);
        goalPostCandidates.push_back(candidate);
        totalValidGoals++;
        matchFound =  " **MATCHED**";
      }
      tlog(50, "Line Pair(%d,%d): dist %f euclid %f whiteFrac %f greenFrac %f %s", i, j, abs(posi.y - posj.y), euclidDist, whitePercentage, greenPct, matchFound.c_str());
    }
  }
  tlog(50, "");
}




/** A hack to see if this post is likely a robot appendage (e.g. an arm)*/
float GoalDetector::filterRobotLimbs(GoalPostCandidate *b){

  int yWSearchSize = 32;    // 20

  int ymin = b->yf;
  int ymax = b->yf + yWSearchSize;
  ymin = std::max(ymin, 0) - (ymin % vstep);
  ymax = std::min(ymax, iparams_.height - 1) - (ymax % vstep);

  int xWSearchSize = 32;    // 30

  // Check left side of base
  int xmin = b->xi - xWSearchSize;
  int xmax = b->xi;
  xmin = std::max(xmin, 0) - (xmin % hstep);
  xmax = std::min(xmax, iparams_.width - 1) - (xmax % hstep);

  int wCount = 0;
  int area = 0;
  for (int y = ymin; y <= ymax; y+=vstep){
    for (int x = xmin; x <= xmax; x+=hstep){
      Color c = color_segmenter_.xy2color(x, y);
      if(c == c_WHITE || c == c_ROBOT_WHITE){
        wCount++;
      }
      area++;
    }
  }

  float leftWhitePct = 1.0 * wCount / area;

  // Check right side of base
  xmin = b->xf;
  xmax = b->xf + xWSearchSize;
  xmin = std::max(xmin, 0) - (xmin % hstep);
  xmax = std::min(xmax, iparams_.width - 1) - (xmax % hstep);

  wCount = 0;
  area = 0;
  for (int y = ymin; y <= ymax; y+=vstep){
    for (int x = xmin; x <= xmax; x+=hstep){
      Color c = color_segmenter_.xy2color(x, y);
      if(c == c_WHITE || c == c_ROBOT_WHITE){
        wCount++;
      }
      area++;
    }
  }

  float rightWhitePct = 1.0 * wCount / area;

  // If neither has green, then reject

  //      if (DEBUG_OUTPUT) printf("Post: left green pct %f right green pct %f\n", leftGreenPct, rightGreenPct);

  return max(leftWhitePct, rightWhitePct);

}


void GoalDetector::detectWhiteGoal(){
  
  int count = 0;

  goalPostCandidates.clear();  
  totalValidGoals = 0;

  if (camera_ == Camera::BOTTOM) return;
  
//  printf("START GOAL LOOP\n");
    
#ifdef TOOL
  cv::Mat wholeImage = color::rawToMat(vblocks_.image->getImgTop(), iparams_);
#endif

  // Obtain ROIs from field edge detector
  for (int o = 0; o < field_edge_detector_.objects.size(); o++){

    VisionObjectCandidate &object = field_edge_detector_.objects[o];

    if (!object.valid || !object.postCandidate)
      continue;
    
   
    // ELAD DOES SOMETHING HERE
    ////////////////////////////

    // Create a candidate for this object
    GoalPostCandidate cand;

    // TODO: xi,xf should account for slant in the image
    // Project (xi,yi) and (xf,yi) to form new box

    int initialXi = max(0, object.xi - hstep * 3);
    int initialXf = min(object.xf + hstep * 3, iparams_.width - 1);

    Coordinates transformXi = transformPointInv(initialXi, object.yf);
    Coordinates transformXf = transformPointInv(initialXf, object.yf);

    // Make sure they land on pixels still.
    transformXi.x -= transformXi.x % hstep;
    transformXf.x += hstep - transformXf.x % hstep;

    // These need to be projected back into classified pixel space

/*    
    Coordinates temp = transformPointInv(0, 0);
    printf("TL (%d,%d)\n", temp.x, temp.y);
    temp = transformPointInv(0, iparams_.height - 1);
    printf("BL (%d,%d)\n", temp.x, temp.y);
    temp = transformPointInv(iparams_.width - 1, 0);
    printf("TR (%d,%d)\n", temp.x, temp.y);
    temp = transformPointInv(iparams_.width - 1, iparams_.height - 1);
    printf("BR (%d,%d)\n", temp.x, temp.y);
*/



    cand.xi = max(0, min(initialXi,transformXi.x));
    cand.xf = min(max(initialXf,transformXf.x), iparams_.width - 1);
    cand.avgX = object.avgX;
    cand.yf = object.yf;
    

//    Position postBase = cmatrix_.getWorldPosition(cand.avgX, cand.yf);
//    Coordinates postTop = cmatrix_.getImageCoordinates(postBase.x, postBase.y, GOAL_HEIGHT);
   
//    printf("POST TOP Y: %d GOAL HEIGHT %f\n", postTop.y, GOAL_HEIGHT);

    int horizonY = horizon_.getYGivenX(cand.avgX);
//    printf("HORIZON: %d\n", horizonY);

    int HORIZON_SLACK = 48;
    horizonY = std::max(0, horizonY - HORIZON_SLACK);

    horizonY -= (horizonY % vstep);

  // TODO: Sanmit. Make sure changing yi from 0 does not have negative repurcussions later...

    cand.yi = horizonY;
    cand.avgY = (cand.yi + cand.yf) / 2.0;


//    printf("Transformed XI (%d) --> (%d)\n", initialXi, cand.xi);
//    printf("Transformed XF (%d) --> (%d)\n\n", initialXf, cand.xf);


    // TODO: when doing extraction, we need to scale the subsampling according to depth in the image

    ROI candROI(cand.xi, cand.xf, cand.yi, cand.yf, camera_);
    candROI.xstep = hstep;
    candROI.ystep = vstep;

    

    // We can use the height of the ROI to guide high res values
    if (cand.yf < 150){
      candROI.setScale(1,1);
    }
    else if (cand.yf < 325){
      candROI.setScale(2,2);
    }
    
    
    int default_hscale = 3;
    int default_vscale = 2;
    if(camera_ == Camera::TOP && (candROI.xstep < hstep || candROI.ystep < vstep)) {
//      tlog(70, "Doing high res scan for post");
      color_segmenter_.setStepScale(candROI.hscale, candROI.vscale);
      const FocusArea focusArea(candROI);
      color_segmenter_.classifyImage(focusArea);
      color_segmenter_.setStepScale(default_hscale, default_vscale);
    }


    //printf("Image Extraction\n");
  
    // Extract images
    cv::Mat mat = color_segmenter_.extractMat(candROI);
    cv::Mat output = color_segmenter_.extractMat(candROI);
    ////////////////////////////////// END ELAD STUB 


    // Goalpost extraction code



    // Extract edges
    // Let's just try canny now since it should be fast enough on small scale 
    cv::Mat edge;
    Canny(mat, edge, 100, 200, 3); // 100-300     // 50 - 200
    cvtColor(edge, output, CV_GRAY2BGR);
    
    
    // Perform Hough transform
    int votesNeeded = 20;    // 20      //50;         // 30
    double minLineLength = mat.rows / 2;    //mat.rows/2
    double maxLineGap = 5; // 5
    double resolution = 1; //1;   // 1 // 5


    vector<cv::Vec4i> lines;
    HoughLinesP(edge, lines, resolution, CV_PI/180, votesNeeded, minLineLength, maxLineGap);  


    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
//        line( output, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
    }





    // TODO: Note that these checks for parallel-ness assume the image is not distorted by using different x and y steps. Otherwise we should reproject first. 
    
    // create groups of lines

    vector<vector<LineSegment>> lineGroups;

/*    if (lines.size() > 0){
      vector<LineSegment> group;
      Vec4i l = lines[0];
      LineSegment line(l[0], l[1], l[2], l[3]); 
      group.push_back(line);
      lineGroups.push_back(group);
      printf("Line %d angle: %f\n", 0, line.getAngle() * RAD_T_DEG); 
    }
*/

    for (int i = 0; i < lines.size(); i++){

      Vec4i l = lines[i];
      LineSegment line(l[0], l[1], l[2], l[3]);

//      printf("Line %d angle: %f\n", i, normalizeAngleNinety(line.getAngle()) * RAD_T_DEG); 
      
      // This line is not vertical enough to be a post candidate. skip it.
      if (fabs(normalizeAngleNinety(line.getAngle())) * RAD_T_DEG < 70) continue; 

      bool isParallel = false;
      // Check if it is parallel with any existing groups
      for (int j = 0; j < lineGroups.size(); j++){

//        printf("Compare line ");
        float angDiff = normalizeAngleNinety(fabs(line.getAngle() - lineGroups[j][0].getAngle())) * RAD_T_DEG;
//        printf("Angdiff: %f\n", angDiff);
        // Add it to that group
//        if (line.isParallelTo(lineGroups[j][0], 1.0)){
        if (fabs(angDiff)  < 10){
          lineGroups[j].push_back(line);
          isParallel = true;
          break;
        }

      }

      if (!isParallel){
        // Create new group
        vector<LineSegment> group;
        group.push_back(line);
        lineGroups.push_back(group);
      }
    }
    
  
//    printf("Total lines: %d, line groups: %d\n\n", lines.size(), lineGroups.size());

  
    // Sort them by x

    //printf("Group stuff\n");

    vector<vector<LineSegment>> unprojSegGroups;

    for (int i = 0; i < lineGroups.size(); i++){
       
      std::sort(lineGroups[i].begin(), lineGroups[i].end(), sortLineSegmentByXF);
//      printf("Group %d\n==> ", i);

      vector<LineSegment> unprojSeg;

      // Reproject the lines back into global image space
      for (int j = 0; j < lineGroups[i].size(); j++){
//        printf("(%1.0f, %1.0f) ", lineGroups[i][j].end.x, lineGroups[i][j].end.y);
  

        LineSegment *line = &lineGroups[i][j];

        LineSegment projLine(line->getXGivenY(0), 0, line->getXGivenY(mat.rows-1), mat.rows-1);

        unprojSeg.push_back(projLine);

        // TODO: what to do if it goes off the edge?

//        printf("Height was %d\n", mat.rows);


//        cv::line( output, cv::Point(projLine.start.x, projLine.start.y), cv::Point(projLine.end.x, projLine.end.y), Scalar(0,0,255), 1, CV_AA);

//        printf("Line in image space: (%f,%f) to (%f,%f)\n", projLine.start.x, projLine.start.y, projLine.end.x, projLine.end.y);

        // Project line back to original image space

//        printf("xmin %d ymin %d xstep %d ystep %d \n", candROI.xmin, candROI.ymin, candROI.xstep, candROI.ystep);

        line->start.x = candROI.xmin + projLine.start.x * candROI.xstep;
        line->start.y = candROI.ymin + projLine.start.y * candROI.ystep;
        line->end.x = candROI.xmin + projLine.end.x * candROI.xstep;
        line->end.y = candROI.ymin + projLine.end.y * candROI.ystep;

        // The above doesn't update the line object parameters...
        lineGroups[i][j] = LineSegment(line->start.x, line->start.y, line->end.x, line->end.y);

#ifdef TOOL        
        cv::line( wholeImage, cv::Point(line->start.x, line->start.y), cv::Point(line->end.x, line->end.y), Scalar(0,0,255), 1, CV_AA);
#endif

//        printf("Line in image space: (%f,%f) to (%f,%f)\n", line->start.x, line->start.y, line->end.x, line->end.y);
/*

        LineSegment projLine;
        projLine.start.x = candROI.xmin + (line->start.x * candROI.xstep);
        projLine.start.y = candROI.ymin + (line->start.y * candROI.ystep);
        projLine.end.x = candROI.xmin + (line->end.x * candROI.xstep);
        projLine.end.y = candROI.ymin + (line->end.y * candROI.ystep);

        printf("project xstart from %f to %f, end from %f to %f\n", line->start.x, projLine.getXGivenY(candROI.ymin), line->end.x, projLine.getXGivenY(candROI.ymax));

        // Project line to bounding box
        line->start.x = projLine.getXGivenY(candROI.ymin);
        line->start.y = candROI.ymin;
        line->end.x = projLine.getXGivenY(candROI.ymax);
        line->end.y = candROI.ymax;
*/      
      }
//      printf("\n");

      unprojSegGroups.push_back(unprojSeg);

    }
//    printf("\n");


    //printf("Line to Post\n");

    
//    printf("%d groups\n", lineGroups.size());
    // Iterate through lines and form goal post candidates.
    for (int i = 0; i < lineGroups.size(); i++){

//      printf("group %d size: %d\n", i, lineGroups[i].size());

      for (int j = 1; j < lineGroups[i].size(); j++){

        LineSegment *leftEdge = &lineGroups[i][j-1];
        LineSegment *rightEdge = &lineGroups[i][j];

        Position posi = cmatrix_.getWorldPosition(leftEdge->end.x, leftEdge->end.y);
        Position posj = cmatrix_.getWorldPosition(rightEdge->end.x, rightEdge->end.y);

//        printf("Left (%0.0f,%0.0f) right (%0.0f,%0.0f)\n", leftEdge->end.x, leftEdge->end.y, rightEdge->end.x, rightEdge->end.y);

        float postWidth = sqrt(pow(posi.x - posj.x, 2) + pow(posi.y -posj.y, 2));

        // This is a crude heuristic to select only one post from each group 
        int center = candROI.xmin + (candROI.xmax - candROI.xmin) / 2.0;
        bool aroundCenter = true; //leftEdge->end.x <= center && rightEdge->end.x >= center;
          
       
        // Calculate edge strength
        float edgeStrength = calculateEdgeBetweenStrength(unprojSegGroups[i][j-1], unprojSegGroups[i][j], edge);


        float topWidth = rightEdge->start.x - leftEdge->start.x;
        float bottomWidth = rightEdge->end.x - leftEdge->end.x;
        float widthRatio = topWidth / bottomWidth;
        bool widthRatioCheck = (widthRatio <= 1.5) && (widthRatio >= 0.5);
//        printf("WIDTH RATIO WAS %f %f %f\n", topWidth, bottomWidth, widthRatio);

        float angleDiff = fabs(normalizeAngleNinety(leftEdge->getAngle() - rightEdge->getAngle())) * RAD_T_DEG;
//        printf("Angle difference: %f\n", angleDiff);        




        float whitePct = calculatePostWhitePercentage(*leftEdge, *rightEdge, candROI.xstep, candROI.ystep);

        // Might want to very slightly relax white threshold. Definitely above 0.7, but that gets robots. 0.8 does make some posts difficult if color table is not good. 
        // Postwidth minimum used to be 30. 
        // Postwidth maximum used to be 225
        if (postWidth < 150 && postWidth > 60 && aroundCenter && whitePct > 0.8 && edgeStrength < 0.3 && widthRatioCheck){

          GoalPostCandidate candidate;
          candidate.xi = leftEdge->end.x; 
          candidate.xf = rightEdge->end.x; 
          candidate.yi = leftEdge->start.y;
          candidate.yf = leftEdge->end.y;
          candidate.avgX = (candidate.xi + candidate.xf) / 2;
          candidate.avgY = (candidate.yi + candidate.yf) / 2;
          candidate.invalid = false;
          candidate.invalidIndex = -1;
//          candidate.edgeSize = bi->edgeSize + bj->edgeSize;
//          candidate.edgeStrength = bi->edgeStrength + bj->edgeStrength;
//          candidate.leftEdgeWidth = bi->xf - bi->xi + 1;
//          candidate.rightEdgeWidth = bj->xf - bj->xi + 1;
          candidate.width = postWidth;
//          candidate.greenBelowPct = greenPct;
//          candidate.whitePct = whitePercentage;
          candidate.objectIndex = o;
          candidate.relPosition = cmatrix_.getWorldPosition(candidate.avgX, candidate.yf);
          goalPostCandidates.push_back(candidate);
          totalValidGoals++;
#ifdef TOOL          
//          printf("Post created. ");
#endif        
        }
        else {
#ifdef TOOL
//          printf("Post failed. ");
#endif        
        }
#ifdef TOOL  
//        printf("Width was: %f aroundCenter %d whitePct %f edge %f\n", postWidth, aroundCenter, whitePct, edgeStrength);
#endif
      }

    }


#ifdef TOOL
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( output, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
    }



//    printf("\n");
    
    // Draw
    std::string filepath = util::ssprintf("/home/sanmit/Desktop/goalROI/img%d.png", count++); 
    cv::imwrite(filepath, output);
#endif

  }

#ifdef TOOL

    std::string wholePath = util::ssprintf("/home/sanmit/Desktop/goalROI/whole.png");
    cv::imwrite(wholePath, wholeImage);

#endif


//  printf("Classifier\n");

  // Filtering


  // Classifier and localization evaluation

  // World Object population
  bool post_is_in_right_spot_according_to_localization = false;
  postCount = 0;
#ifdef TOOL
      vblocks_.roi->goal_rois_.clear();
#endif
  if (VisionCore::isStreaming() && vblocks_.roi->log_block){
    vblocks_.roi->goal_rois_.clear();
  }
  for (int i = 0; postCount < 2 && i < goalPostCandidates.size(); i++){
    const auto& cand = goalPostCandidates[i];
    if (cand.invalid) continue;
    if (VisionCore::isStreaming() && vblocks_.roi->log_block){
      ROI roi(cand.xi, cand.xf, cand.yi, cand.yf, camera_);
      int width = roi.width();
      int height = roi.height();
      roi.xmin = std::max<int>(0, roi.xmin - width/4);
      roi.xmax = std::min<int>(iparams_.width-1, roi.xmax + width/4);
      roi.ymax = std::min<int>(iparams_.height-1, roi.ymax + height/10);
      if(roi.area() < 75 * 200) { // High res if it's small enough
        roi.xstep = roi.ystep = 1;
        color_segmenter_.extractMat(roi, ColorSegmenter::Source::HighresGrayscale);
      } else {
        color_segmenter_.getStepSize(roi.xstep, roi.ystep);
        color_segmenter_.extractMat(roi);
      }
      auto size = cv::Size(16, 64);
      cv::Mat resized;
      cv::resize(roi.mat, resized, size);
      roi.mat = resized;
      vblocks_.roi->goal_rois_.push_back(roi.clone());
    }
    bool classifier_says_is_post = true;

    // TODO: Fix classifier.

//    if(classifier_ != nullptr) {
    if (false){
      ROI roi(cand.xi, cand.xf, cand.yi, cand.yf, camera_);
      int width = roi.width();
      int height = roi.height();
      roi.xmin = std::max<int>(0, roi.xmin - width/4);
      roi.xmax = std::min<int>(iparams_.width-1, roi.xmax + width/4);
      roi.ymax = std::min<int>(iparams_.height-1, roi.ymax + height/10);
      VisionTimer::Start(82, "deep goal classifier");
      tlog(50, "extracting matrix for post %i", i);
      VisionTimer::Wrap(82, "deep goal classifier: extract");
      if(roi.area() < 75 * 200) { // High res if it's small enough
        roi.xstep = roi.ystep = 1;
        color_segmenter_.extractMat(roi, ColorSegmenter::Source::HighresGrayscale);
      } else {
        color_segmenter_.getStepSize(roi.xstep, roi.ystep);
        color_segmenter_.extractMat(roi);
      }
      VisionTimer::Wrap(82, "deep goal classifier: extract");
      auto size = cv::Size(16, 64);
      tlog(51, "resizing to %ix%i", size.width, size.height);
      cv::Mat resized;
      cv::resize(roi.mat, resized, size);
      roi.mat = resized;
#ifdef TOOL
      vblocks_.roi->goal_rois_.push_back(roi.clone());
#endif
      float confidence;
      VisionTimer::Wrap(82, "deep goal classifier: classify");
      bool result = classifier_->classify(roi.mat, confidence);
      VisionTimer::Wrap(82, "deep goal classifier: classify");
      tlog(50, "classifier gave result %s with confidence %2.4f", result ? "GOAL" : "NOT GOAL", confidence);
      VisionTimer::Stop(82, "deep goal classifier");
      classifier_says_is_post = result;
    }

    
    if (post_is_in_right_spot_according_to_localization || classifier_says_is_post){
      posts[postCount++] = goalPostCandidates[i];
    }
    else {
      goalPostCandidates[i].invalid = true;
      goalPostCandidates[i].invalidIndex = 11;
      totalValidGoals--;
    }
  }
    
    // TODO: Also, if both post candidates are getting matched to the same post, then we should throw one of them out. Maybe the one that is not as close of a match. This is needed because sometimes we'll split an actual post into two post candidates, and both of them seem reasonable, but are too close together.

    // Turn on detectgoalposts in image processor

  tlog(50, "%d goal posts found after classifier verification", postCount);
  formWhiteGoal();




  
  // Update objects array from field edges to claim posts  
  for (int i = 0; i < goalPostCandidates.size(); i++){

    if (goalPostCandidates[i].invalid) continue;

    VisionObjectCandidate &object = field_edge_detector_.objects[goalPostCandidates[i].objectIndex];
    object.robotCandidate = false;

  }

//#if defined(TOOL) && defined(USER_sanmit)
  totalPosts += postCount; //goalPostCandidates.size(); //postCount;
//  printf("Total goals found: %d\n", totalPosts);
//#endif

//  if (goalPostCandidates.size() > 0){
//    printf("%d goal post candidates detected\n", goalPostCandidates.size());
//  }

//  printf("END LOOP\n");

}

float GoalDetector::calculateEdgeBetweenStrength(LineSegment &line1, LineSegment &line2, cv::Mat &mat){

  int total = 0;
  int count = 0;

  int midY = line1.start.y + ((line1.end.y - line1.start.y) / 2.0);

  for (int y = line1.start.y; y < midY; y++){

    int xStart = line1.getXGivenY(y);
    int xEnd = line2.getXGivenY(y);

    for (int x = xStart + 1; x < xEnd; x++){
      total++;
      if (mat.at<uchar>(y, x) == 255){
        count++;
      }
    }

  }
  float strength = (float) (count) / (float) (total);
  if (!total)
    strength = 1;

  total = 0;
  count = 0;

  for (int y = midY; y <= line1.end.y; y++){

    int xStart = line1.getXGivenY(y);
    int xEnd = line2.getXGivenY(y);

    for (int x = xStart + 1; x < xEnd; x++){
      total++;
      if (mat.at<uchar>(y, x) == 255){
        count++;
      }
    }

  }


  float strength2 = (float) (count) / (float) (total);
  if (!total)
    strength2 = 1;
  return std::max(strength, strength2);
}



bool sortLineSegmentByXF(LineSegment line1, LineSegment line2){
  return line1.end.x < line2.end.x;
}

Coordinates GoalDetector::transformPointInv(int x, int y) {

  Eigen::Vector3f p, proj_p;
  p << x, y, 1;
  proj_p = H*p;
  float x2,y2;
  x2 = proj_p(0)/proj_p(2);
  y2 = proj_p(1)/proj_p(2);
  return Coordinates(x2,y2);

}


// Must have called this already
// houghdetector->setimagepointers
// houghdetector->setHorizon
void GoalDetector::detectGoalPosts(){

  totalValidGoals = 0;
  goalPostCandidates.clear();

  if (camera_ == Camera::BOTTOM) return;

  if (!hough_detector_.horizonSet) return;    // We fell down. Don't process this frame

  hough_detector_.extractLines(); 


  tlog(50, "Detecting goal posts\n");




  // Form inital goal post candidates from vertical hough lines (populates goalPostCandidates vector)
  formGoalPostCandidates(hough_detector_.vertLines);



  filterPosts();


  // TODO: If there are more than 2 candidates at this point, we need to decide which ones to run the classifier on. 
  tlog(50, "%d valid goal posts found after filtering\n", totalValidGoals);
  if (totalValidGoals > 2){
    tlog(50, "Found too many posts. Throwing all out.");
    for (int i = 0; i < goalPostCandidates.size(); i++){
      auto& cand = goalPostCandidates[i];
      if (!cand.invalid){
        cand.invalid = true;
        cand.invalidIndex = 10;
        totalValidGoals--;
      }
    }
  }

/*
  if (totalValidGoals == 2){
    for (int i = 0; i < goalPostCandidates.size(); i++){
      if (goalPostCandidates[i].invalid) continue; 
      for (int j = i+1; j < goalPostCandidates.size(); j++){
        if (goalPostCandidates[j].invalid) continue;
        Position post1Pos = goalPostCandidates[i].relPosition;
        Position post2Pos = goalPostCandidates[j].relPosition;
        float distBetweenPosts = sqrt(pow(post1Pos.x - post2Pos.x, 2) + pow(post1Pos.y -post2Pos.y, 2));
        tlog(50, "Distance between post candidates: %f", distBetweenPosts);
        break;
      }
    }
  }
*/

  
  // I think we will skip this now that we are using a classifier
//  formGoalFromPosts();




// ***************************************  

// TODO: JAKE
  
  bool post_is_in_right_spot_according_to_localization = false;
  postCount = 0;
#ifdef TOOL
      vblocks_.roi->goal_rois_.clear();
#endif
  if (VisionCore::isStreaming() && vblocks_.roi->log_block){
    vblocks_.roi->goal_rois_.clear();
  }
  for (int i = 0; postCount < 2 && i < goalPostCandidates.size(); i++){
    const auto& cand = goalPostCandidates[i];
    if (cand.invalid) continue;
    if (VisionCore::isStreaming() && vblocks_.roi->log_block){
      ROI roi(cand.xi, cand.xf, cand.yi, cand.yf, camera_);
      int width = roi.width();
      int height = roi.height();
      roi.xmin = std::max<int>(0, roi.xmin - width/4);
      roi.xmax = std::min<int>(iparams_.width-1, roi.xmax + width/4);
      roi.ymax = std::min<int>(iparams_.height-1, roi.ymax + height/10);
      if(roi.area() < 75 * 200) { // High res if it's small enough
        roi.xstep = roi.ystep = 1;
        color_segmenter_.extractMat(roi, ColorSegmenter::Source::HighresGrayscale);
      } else {
        color_segmenter_.getStepSize(roi.xstep, roi.ystep);
        color_segmenter_.extractMat(roi);
      }
      auto size = cv::Size(16, 64);
      cv::Mat resized;
      cv::resize(roi.mat, resized, size);
      roi.mat = resized;
      vblocks_.roi->goal_rois_.push_back(roi.clone());
    }
    bool classifier_says_is_post = true;
    if(classifier_ != nullptr) {
      ROI roi(cand.xi, cand.xf, cand.yi, cand.yf, camera_);
      int width = roi.width();
      int height = roi.height();
      roi.xmin = std::max<int>(0, roi.xmin - width/4);
      roi.xmax = std::min<int>(iparams_.width-1, roi.xmax + width/4);
      roi.ymax = std::min<int>(iparams_.height-1, roi.ymax + height/10);
      VisionTimer::Start(82, "deep goal classifier");
      tlog(50, "extracting matrix for post %i", i);
      VisionTimer::Wrap(82, "deep goal classifier: extract");
      if(roi.area() < 75 * 200) { // High res if it's small enough
        roi.xstep = roi.ystep = 1;
        color_segmenter_.extractMat(roi, ColorSegmenter::Source::HighresGrayscale);
      } else {
        color_segmenter_.getStepSize(roi.xstep, roi.ystep);
        color_segmenter_.extractMat(roi);
      }
      VisionTimer::Wrap(82, "deep goal classifier: extract");
      auto size = cv::Size(16, 64);
      tlog(51, "resizing to %ix%i", size.width, size.height);
      cv::Mat resized;
      cv::resize(roi.mat, resized, size);
      roi.mat = resized;
#ifdef TOOL
      vblocks_.roi->goal_rois_.push_back(roi.clone());
#endif
      float confidence;
      VisionTimer::Wrap(82, "deep goal classifier: classify");
      bool result = classifier_->classify(roi.mat, confidence);
      VisionTimer::Wrap(82, "deep goal classifier: classify");
      tlog(50, "classifier gave result %s with confidence %2.4f", result ? "GOAL" : "NOT GOAL", confidence);
      VisionTimer::Stop(82, "deep goal classifier");
      classifier_says_is_post = result;
    }

    
    if (post_is_in_right_spot_according_to_localization || classifier_says_is_post){
      posts[postCount++] = goalPostCandidates[i];
    }
    else {
      goalPostCandidates[i].invalid = true;
      goalPostCandidates[i].invalidIndex = 11;
      totalValidGoals--;
    }
  }
    
    // TODO: Also, if both post candidates are getting matched to the same post, then we should throw one of them out. Maybe the one that is not as close of a match. This is needed because sometimes we'll split an actual post into two post candidates, and both of them seem reasonable, but are too close together.

    // Turn on detectgoalposts in image processor

  tlog(50, "%d goal posts found after classifier verification", postCount);
  formWhiteGoal();


#if defined(TOOL) && defined(USER_sanmit)
  totalPosts += postCount;
  printf("Total goals found: %d\n", totalPosts);
#endif
}


void GoalDetector::filterPosts(){
  
  
  
  int goalLineIndex1, goalLineIndex2;
  bool foundParallelLines = findGoalBox(goalLineIndex1, goalLineIndex2);
  LineSegment goalLine1, goalLine2;
  if (foundParallelLines){
    goalLine1 = vblocks_.world_object->objects_[goalLineIndex1].visionLine;
    goalLine2 = vblocks_.world_object->objects_[goalLineIndex2].visionLine;
  }

  tlog(50, "Filtering %d post candidates", goalPostCandidates.size());

  for (int i = 0; i < goalPostCandidates.size(); i++){
    GoalPostCandidate *b = &goalPostCandidates[i];
    Position pos = cmatrix_.getWorldPosition(b->avgX, b->yf);

    Point2D postPos2D(pos.x, pos.y);



    float distByWidth = cmatrix_.getWorldDistanceByWidth(b->xf - b->xi + 1, GOAL_POST_WIDTH);
    float groundDist = cmatrix_.groundDistance(pos);
    float directDist = cmatrix_.directDistance(pos);


    // 0.3
    float onGreenPct = getOnGreenPercentage(b);
    if ( onGreenPct < 0.3){
      b->invalid = true;
      b->invalidIndex = 0;
      totalValidGoals--;
      tlog(50, "Invalidated candidate %d: green next to base percentage was %f", i, onGreenPct);
      continue;
    }

    if (foundParallelLines){
      float distToLine = min(postPos2D.getDistanceTo(goalLine1.getPointOnLineClosestTo(postPos2D)), postPos2D.getDistanceTo(goalLine2.getPointOnLineClosestTo(postPos2D)));
      // 100
      if (distToLine > 150){
        b->invalid = true;
        b->invalidIndex = 1;
        totalValidGoals--;
        tlog(50, "Invalidated candidate %d: parallel lines found, but dist to line was %f", i, distToLine);
        continue;
      }
    }

    float dToClosestLine = distToClosestLine(b);

    if ( cmatrix_.groundDistance(b->relPosition) < 1000 && dToClosestLine > 150){
      b->invalid = true;
      b->invalidIndex = 1;
      totalValidGoals--;
      tlog(50, "Invalidated candidate %d: post is close but not on a line (%f away)", i, dToClosestLine);
      continue;
    }


    // Check edge strength density between lines. Make sure it's not too much, otherwise this could be a robot. Posts should be "smooth." 
    int size;
    int strength;
    int maxStrength;
    hough_detector_.getEdgeData(b->xi + b->leftEdgeWidth, b->xf - b->rightEdgeWidth, b->yi, b->yf, size, strength, maxStrength); 

    // This is the average intensity of the pixels between the goal post edges. Ranges from 0 to 255. 
    float strengthBetween = 1.0 * strength / size;

    if (strengthBetween > 110){  // 90
      b->invalid = true;
      b->invalidIndex = 2;
      totalValidGoals--;
      tlog(50, "Invalidated candidate %d: edge strength between post edges too high %f", i, strengthBetween);
      continue;
    }

    // TODO.    
    float robotWhitePct = filterRobotLimbs(b);
    if (false) { // robotWhitePct > 0.45){
      b->invalid = true;
      b->invalidIndex = 3;
      totalValidGoals--;
      tlog(50, "Invalidated candidate %d: likely to be a robot. White pct %f", i, robotWhitePct);
      continue;
    }

    //    if (abs(groundDist - distByWidth) > 1500){
    //      b->invalid = true;
    //      if(DEBUG_OUTPUT) printf("Invalidated candidate %d: groundDist: %f, widthDist: %f, absDiff: %f\n", i, groundDist, distByWidth, abs(groundDist - distByWidth));
    //      continue;
    //    }

    tlog(50, "Valid candidate %d: greenBase %f found parallel %d absDiffDistance %f strengthBetween %f maxStrength %f robotWhitePct %f", i, getOnGreenPercentage(b), foundParallelLines, abs(groundDist - distByWidth), 1.0 * strength / size, 1.0 * maxStrength / size, robotWhitePct);

    }

    tlog(50, "");

  }


  void GoalDetector::formGoalFromPosts(){

    vector<GoalCandidate> goalCandidates;
    postCount = 0;


    tlog(50, "%d goal post candidates\n", goalPostCandidates.size());


    // Check all pairs to see if any are reasonable for a goal
    bool foundMatchingPosts = false;
    for (int i = 0; i < (int)goalPostCandidates.size() - 1; i++){


      if (goalPostCandidates[i].invalid)
        continue;

      //      Position post1Pos = cmatrix_.getWorldPosition(goalPostCandidates[i].avgX, goalPostCandidates[i].yf); 

      Position post1Pos = goalPostCandidates[i].relPosition;

      for (int j = i+1; j < goalPostCandidates.size(); j++){

        if (goalPostCandidates[j].invalid)
          continue;

        //        Position post2Pos = cmatrix_.getWorldPosition(goalPostCandidates[j].avgX, goalPostCandidates[j].yf); 

        Position post2Pos = goalPostCandidates[j].relPosition;

        float distBetweenPosts = sqrt(pow(post1Pos.x - post2Pos.x, 2) + pow(post1Pos.y -post2Pos.y, 2));

        tlog(50, "Dist between goal post %d and %d: just y %f euclidean %f\n", i, j, abs(post1Pos.y - post2Pos.y), distBetweenPosts);

        if (abs(distBetweenPosts - GOAL_WIDTH) < 1000){
          foundMatchingPosts = true;
          posts[0] = goalPostCandidates[i];
          posts[1] = goalPostCandidates[j];
          postCount = 2;

          GoalCandidate gCand;
          gCand.width = distBetweenPosts;
          if (goalPostCandidates[i].avgX < goalPostCandidates[j].avgX){
            gCand.leftPost = goalPostCandidates[i];
            gCand.rightPost = goalPostCandidates[j];
          }
          else {
            gCand.leftPost = goalPostCandidates[j];
            gCand.rightPost = goalPostCandidates[i];
          }
          goalCandidates.push_back(gCand);
        }

      }

    }


    if (foundMatchingPosts){
      std::sort(goalCandidates.begin(), goalCandidates.end(), sortGoalPredicate);
      posts[0] = goalCandidates[0].leftPost;
      posts[1] = goalCandidates[0].rightPost;
    }


    // Otherwise we'll just take the one with highest edge strength
    if (!foundMatchingPosts && goalPostCandidates.size() > 0){
      std::sort(goalPostCandidates.begin(), goalPostCandidates.end(), sortPostEdgeStrengthPredicate);
      posts[postCount++] = goalPostCandidates[0];
    }


  }


void GoalDetector::formWhiteGoal(){

  // TODO: calculate vision distance using something similar to estimateGoalDistance()


  if (postCount < 1 || postCount > 2){
    return;
  }
//  tlog(50, "Found %d posts\n", postCount); 


  if (postCount == 1){

    Position post1Pos = posts[0].relPosition;
    float d1 = cmatrix_.groundDistance(post1Pos);
    float b1 = cmatrix_.bearing(post1Pos);
    float e1 = cmatrix_.elevation(post1Pos);


    // Attempt to find which goal post it is based on intersections
    int postIndex = WO_UNKNOWN_GOALPOST;
    // Populate just the one
    setGoalObject(postIndex, d1, b1, e1, posts[0].avgX, posts[0].yf, 1, -1);
  }


  else {

    Position post1Pos = posts[0].relPosition; 
    float d1 = cmatrix_.groundDistance(post1Pos);
    float b1 = cmatrix_.bearing(post1Pos);
    float e1 = cmatrix_.elevation(post1Pos);


    Position post2Pos = posts[1].relPosition;
    float d2 = cmatrix_.groundDistance(post2Pos);
    float b2 = cmatrix_.bearing(post2Pos);
    float e2 = cmatrix_.elevation(post2Pos);

    // Populate both world objects

    float bearing = (b1 + b2) / 2, elevation = (e1 + e2) / 2;
    float confidence = 1, confidence1 = 1, confidence2 = 1;

    // Decide which post is left and which post is right
    Position left = post1Pos, right = post2Pos;
    int post1 = WO_UNKNOWN_LEFT_GOALPOST;
    int post2 = WO_UNKNOWN_RIGHT_GOALPOST;
    if (posts[1].avgX < posts[0].avgX) {
      post1 = WO_UNKNOWN_RIGHT_GOALPOST;
      post2 = WO_UNKNOWN_LEFT_GOALPOST;
      left = post2Pos;
      right = post1Pos;
    }

    // TODO: See formGoal for distance calculations 
    float distancePosts = estimateGoalDistanceByPosts(left, right);
    //    float distanceAvg = (headDistance1 + headDistance2) / 2;
    //    float distance = (distancePosts + distanceAvg) / 2;
    float distance = distancePosts;


    // Set Goals and Goal Posts
    setGoalObject(WO_UNKNOWN_GOAL, distance, bearing, elevation,
        (posts[0].avgX + posts[1].avgX) / 2,
        (posts[0].avgY + posts[1].avgY) / 2,
        confidence, -1);
    setGoalObject(post1, d1, b1, e1,
        posts[0].avgX, posts[0].yf,
        confidence1, -1);
    setGoalObject(post2, d2, b2, e2,
        posts[1].avgX, posts[1].yf,
        confidence2, -1);
  }

}

    float GoalDetector::distToClosestLine(GoalPostCandidate *post){

      float dist = 10000;
      Point2D postPos2D(post->relPosition.x, post->relPosition.y);

      for ( int line = WO_UNKNOWN_FIELD_LINE_1; line < WO_UNKNOWN_FIELD_LINE_4 && vblocks_.world_object->objects_[line].seen; line++){
        LineSegment lineSeg = vblocks_.world_object->objects_[line].visionLine;
        float distToLine = postPos2D.getDistanceTo(lineSeg.getPointOnLineClosestTo(postPos2D));
        if (distToLine < dist){
          dist = distToLine;
        }
      }

      return dist;
    }

    bool GoalDetector::findGoalBox(int &line1, int &line2){

      int firstLine = WO_UNKNOWN_FIELD_LINE_1;
      int lastLine = WO_UNKNOWN_FIELD_LINE_4;

      for (line1 = firstLine; line1 < lastLine && vblocks_.world_object->objects_[line1].seen; line1++){

        Point2D line1Pt1 = vblocks_.world_object->objects_[line1].visionPt1;
        //    Position line1Pt1 = cmatrix_.getWorldPosition(vline1Pt1.x, vline1Pt1.y);
        Point2D line1Pt2 = vblocks_.world_object->objects_[line1].visionPt2;
        //    Position line1Pt2 = cmatrix_.getWorldPosition(vline1Pt2.x, vline1Pt2.y);
        float line1Slope = 1.0 * (line1Pt2.y - line1Pt1.y) / (line1Pt2.x - line1Pt1.x);
        float line1Intercept = (-line1Slope * line1Pt1.x) + line1Pt1.y;

        Position line1Midpoint((line1Pt1.x + line1Pt2.x) * 0.5, 0.5 * (line1Pt1.y + line1Pt2.y), 0);

        for (line2 = line1+1; line2 <= lastLine && vblocks_.world_object->objects_[line2].seen; line2++){

          Point2D line2Pt1 = vblocks_.world_object->objects_[line2].visionPt1;
          //      Position line2Pt1 = cmatrix_.getWorldPosition(vline2Pt1.x, vline2Pt1.y);
          Point2D line2Pt2 = vblocks_.world_object->objects_[line2].visionPt2;
          //      Position line2Pt2 = cmatrix_.getWorldPosition(vline2Pt2.x, vline2Pt2.y);
          float line2Slope = 1.0 * (line2Pt2.y - line2Pt1.y) / (line2Pt2.x - line2Pt1.x);

          float line2Intercept = (-line2Slope * line2Pt1.x) + line2Pt1.y;

          Position line2Midpoint(0.5 * (line2Pt1.x + line2Pt2.x), 0.5 * (line2Pt1.y + line2Pt2.y), 0);

          // Check that they aren't the same line -- because line detection doesn't do that...

          float distBetweenLines = sqrt(pow(line1Midpoint.x - line2Midpoint.x, 2) + pow(line1Midpoint.y - line2Midpoint.y, 2));
          float slopeBetweenLines = 1.0 * (line2Midpoint.y - line1Midpoint.y) / (line2Midpoint.x - line1Midpoint.x);


          tlog(50, "Line %d (%f,%f) (%f,%f): slope %f intercept %f -- Line %d (%f,%f) (%f,%f): slope %f intercept %f\n\tabsSlopeDiff %f\n\tabsInterpDiff %f\n\tdistBetween %f slopeBetween %f \n", line1-firstLine, line1Pt1.x, line1Pt1.y, line1Pt2.x, line1Pt2.y, line1Slope, line1Intercept, line2-firstLine, line2Pt1.x, line2Pt1.y, line2Pt2.x, line2Pt2.y, line2Slope, line2Intercept, abs(line1Slope - line2Slope), abs(line1Intercept - line2Intercept), distBetweenLines, slopeBetweenLines); 

          // Technically the goal box is 600mm wide. 
          if (abs(line1Slope - line2Slope) < 0.2 && abs(line1Intercept - line2Intercept) > 100){
            return true;
          }
        }
      }


      return false;


    }



float GoalDetector::getOnGreenPercentage(GoalPostCandidate *b){
  Position pos = cmatrix_.getWorldPosition(b->avgX, b->yf);
  float groundDist = cmatrix_.groundDistance(pos);

  int yGSearchSize = (-0.01 * groundDist) + 55; //20;

  // Step function. If we are farther than 5.5m, just use 15 search area. Chances are we won't see it anyways though.
  if (yGSearchSize < 15){
    yGSearchSize = 15;
  }

  int xGSearchSize = 30;

  int ymin = b->yf - yGSearchSize;
  int ymax = b->yf;
  ymin = std::max(ymin, 0) - (ymin % vstep);
  ymax = std::min(ymax, iparams_.height - 1) - (ymax % vstep);


  // Check left side of base
  int xmin = b->xi - xGSearchSize;
  int xmax = b->xi;
  xmin = std::max(xmin, 0) - (xmin % hstep);
  xmax = std::min(xmax, iparams_.width - 1) - (xmax % hstep);

  int gCount = 0;
  int area = 0;
  tlog(51, "scanning for green from x: %i --> %i, y: %i --> %i with hstep %i vstep %i", xmin, xmax, ymin, ymax, hstep, vstep);
  for (int y = ymin; y <= ymax; y+=vstep){
    for (int x = xmin; x <= xmax; x+=hstep){
      Color c = color_segmenter_.xy2color(x, y);
      if(c == c_FIELD_GREEN){
        gCount++;
      }
      area++;
    }
  }

  //      float leftGreenPct;
  //      
  //      if (!area)
  //        leftGreenPct = 1;
  //      else
  float leftGreenPct = 1.0 * gCount / area;

  // Check right side of base
  xmin = b->xf;
  xmax = b->xf + xGSearchSize;
  xmin = std::max(xmin, 0) - (xmin % hstep);
  xmax = std::min(xmax, iparams_.width - 1) - (xmax % hstep);

  gCount = 0;
  area = 0;
  for (int y = ymin; y <= ymax; y+=vstep){
    for (int x = xmin; x <= xmax; x+=hstep){
      Color c = color_segmenter_.xy2color(x, y);
      if(c == c_FIELD_GREEN){
        gCount++;
      }
      area++;
    }
  }
  //      float rightGreenPct;

  //      if (!area)
  //        rightGreenPct = 1;
  //      else 
  float rightGreenPct = 1.0 * gCount / area;

  // If neither has green, then reject

  //      if (DEBUG_OUTPUT) printf("Post: left green pct %f right green pct %f\n", leftGreenPct, rightGreenPct);

  return max(leftGreenPct, rightGreenPct);

}



    /**
     * Estimates the distance of the goal from the robot given the positions of the
     * two goal posts.
     * @param The positions of the left and right goal posts.
     * @return The distance of the goal as estimated by the goal posts.
     */
    float GoalDetector::estimateGoalDistanceByPosts(Position left, Position right) {
      float kdist = (cmatrix_.groundDistance(left) + cmatrix_.groundDistance(right)) / 2.0f;
      float apparentWidth = (left - right).abs();

      // When we assume the goal is farther away, we over-estimate its world width.
      float cdist = kdist * GOAL_WIDTH / apparentWidth;
      tlog(55, "Both posts found, computing distance from apparent width: %2.f ( / %2.f = %2.2f), kdist: %2.f, corrected dist: %2.f", apparentWidth, GOAL_WIDTH, GOAL_WIDTH / apparentWidth, kdist, cdist);
      return cdist;
    }

    float GoalDetector::estimateGoalDistance(FieldLine * goal) {
      float y1 = goal->tL.y;
      float x1 = (int) ((y1 - goal->Offset) / goal->Slope);
      float y2 = goal->bR.y;
      float x2 = (int) ((y2 - goal->Offset) / goal->Slope);
      float x = x1 - x2, y = y1 - y2;

      float height = sqrtf(x * x + y * y);
      float width = fabs(goal->preciseWidth * sinf(goal->Angle));

      float wdist = cmatrix_.getWorldDistanceByWidth(width, GOAL_POST_WIDTH);
      float kdist = estimateGoalDistanceByKinematics(goal);

      // Width dist is bad at long distances due to the fact that goal posts are relatively thin.
      float wmin = 10.0f, wmax = 40.0f;
      float wratio = std::max(0.0f, std::min(1.0f, (width - wmin) / (wmax - wmin)));
      float kratio = 1.0f - wratio;
      float avg = (wdist * wratio + kdist * kratio) / (wratio + kratio);

      //if(DEBUG_OUTPUT) printf("gw pct : %2.2f, width dist(%2.2f): %2.2f, height dist(%2.2f): %2.2f, kinematics dist(%2.2f,%2.2f): %2.2f, avg: %2.2f\n", greenWhite, width, wdist, height, hdist, goal->bR.x, goal->bR.y, kdist, avg);
      tlog(55,"width dist(%2.2f): %2.2f, height dist(%2.2f): %2.2f, kinematics dist(%2.2f,%2.2f): %2.2f, avg: %2.2f", width, wdist, height, 0.0, goal->bR.x, goal->bR.y, kdist, avg);

      return avg;
    }

    float GoalDetector::estimateGoalDistanceByKinematics(FieldLine * goal) {
      Position p = cmatrix_.getWorldPosition(goal->bR.x, goal->bR.y);
      float dist = cmatrix_.groundDistance(p);
      return dist;
    }

    float GoalDetector::estimateGoalDistanceByHeight(float height) {
      height *= 480.0f / iparams_.height; // Normalize because this was tuned with a height of 480
      // piyushk 6/14/12
      //return 1243031.764766 * powf(height,-1.235303);
      // sbarrett 6/17/12
      return 863523.709600 * powf(height,-1.183317);
    }

    float GoalDetector::estimateGoalDistanceByWidth(float width) {
      width *= 640.0f / iparams_.width; // Normalize because this was tuned with a width of 640
      // piyushk 6/14/12
      //return 38438.155815 * powf(width,-0.905797);
      // sbarrett 6/17/13
      //return 21203.039034 * powf(width,-0.755975);
      // sbarrett 6/17/13 take 2
      return 47611.197494 * powf(width,-0.978952);
    }




    // TODO: Since we don't use confidence, right now I'm using it for width distance to do some logging...

    void GoalDetector::setGoalObject(int goalIndex, float distance, float bearing, float elevation, int centerX, int centerY, float confidence, int lineIndex) {
      vblocks_.world_object->objects_[goalIndex].seen = true;
      vblocks_.world_object->objects_[goalIndex].visionBearing = bearing;
      vblocks_.world_object->objects_[goalIndex].visionElevation = elevation;
      vblocks_.world_object->objects_[goalIndex].visionDistance = distance;
      vblocks_.world_object->objects_[goalIndex].imageCenterX = centerX;
      vblocks_.world_object->objects_[goalIndex].imageCenterY = centerY;
      vblocks_.world_object->objects_[goalIndex].frameLastSeen = vblocks_.frame_info->frame_id;
      // HACK HERE  
      vblocks_.world_object->objects_[goalIndex].visionConfidence = 1.0; //confidence;
      //  vblocks_.world_object->objects_[goalIndex].fieldLineIndex = lineIndex;
      vblocks_.world_object->objects_[goalIndex].fromTopCamera = (camera_ == Camera::TOP);

      /*
         if (ENABLE_CALIBRATION){
         printf("\nGOALPOST DISTANCE: %f\n", distance);
      //    ofstream goalFile;
      //    goalFile.open ("/home/nao/logs/calibration/goalCalibration.txt", ios::out | ios::app);
      //    goalFile << distance << "\n";
      //    goalFile << post1 << " " << d1 << " " << cmatrix_.getWorldDistanceByWidth(posts[0].xf - posts[0].xi + 1, GOAL_POST_WIDTH) << "\n";
      //    goalFile << post2 << " " << d2 << " " << cmatrix_.getWorldDistanceByWidth(posts[1].xf - posts[1].xi + 1, GOAL_POST_WIDTH) << "\n";
      //    goalFile.close();
      }
       */
    }


