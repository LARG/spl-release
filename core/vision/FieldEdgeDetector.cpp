#include "FieldEdgeDetector.h"

//using namespace cv;

FieldEdgeDetector::FieldEdgeDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter, BlobDetector& blob_detector) : DETECTOR_INITIALIZE, color_segmenter_(segmenter), blob_detector_(blob_detector) {


  color_segmenter_.getStepSize(hstep,vstep);
  
  fieldEdgePoints = new int[iparams_.width / hstep];

  srand (time(NULL));

}

/** Main function that detects field edges */
bool FieldEdgeDetector::detectFieldEdges(){


  if (camera_ == Camera::BOTTOM) return false;

//  printf("FIELD EDGE LOOP\n");

//  m_img = color::rawToMat(vblocks_.image->getImgTop(), iparams_);

  findFieldEdgePoints();
  
/*  // Fit lines on left and right 
  Line2D left;
  
  if (ransacLine(0, (4* iparams_.width) /(10 * hstep), left)){
    line(m_img, cv::Point(0, left.getYGivenX(0)), cv::Point(iparams_.width - 1, left.getYGivenX(iparams_.width - 1)), Scalar(0, 255, 0), 2);
  }


  Line2D right;
  if (ransacLine( (6 * iparams_.width) / (10 * hstep), (iparams_.width / hstep) - 1, right)){
    line(m_img, cv::Point(0, right.getYGivenX(0)), cv::Point(iparams_.width - 1, right.getYGivenX(iparams_.width - 1)), Scalar(0, 0, 255), 2);
  }

  imwrite("/home/eladlieb/Desktop/fieldEdge.png", m_img);
*/

//  printf("Finding objects\n");


/* TODO V6-UPDATE -- RE-ENABLE THIS STUFF

  findPossibleObjects();


//  printf("Filtering objects\n");
  filterObjects();

//  printf("Sorting.\n");

  sortObjects();

//  printf("END FIELD EDGE LOOP\n");
*/
  return true;
}

void FieldEdgeDetector::sortObjects(){

  // 

  const float POST_DISTANCE_THRESH = BORDER_STRIP_WIDTH + (0.9 * PENALTY_X);

  for (int i = 0; i < objects.size(); i++){

    if (!objects[i].valid) continue;

    objects[i].robotCandidate = true;

//    printf("Object at (%d,%d) distToEdge: %f\n", objects[i].avgX, objects[i].avgY, objects[i].distToEdge);

    if (objects[i].distToEdge < POST_DISTANCE_THRESH){
      objects[i].postCandidate = true;
    }

  }
//  printf("\n");
    

}


/** Finds the top-most green pixel using green vertical runs, and places in fieldEdgePoints as a possible field edge point candidate */
bool FieldEdgeDetector::findFieldEdgePoints(){
 
  //printf("Finding field edge points\n");

// TODO: need to initialize text logger before using this
//  tlog(80, "Finding field edge points");



  VisionTimer::Start(80, "FieldEdgeDetector(%s)::findFieldEdgePoints", camera_);

  VisionPoint ***verticalPoint = color_segmenter_.verticalPoint;
  uint32_t **verticalPointCount = color_segmenter_.verticalPointCount;

  int index = 0;

//  printf("hstep %d vstep %d\n", hstep, vstep);




  // Precompute cmatrix distances

  const int xjump = 160;
  const int yjump = 80;


  VisionTimer::Start(80, "FieldEdgeDetector(%s)::roughCmatrix", camera_);
  Position positions[13][9];
  for (int row = 0; row < 13; row++){
    for (int col = 0; col < 9; col++){
      positions[row][col] = cmatrix_.getWorldPosition(col * xjump, row * yjump); 
    }
  }
  VisionTimer::Stop("FieldEdgeDetector(%s)::roughCmatrix", camera_);




  VisionTimer::Start(80, "FieldEdgeDetector(%s)::iterateX", camera_);

  // Iterate over X
  for (int x = 0; x < iparams_.width; x += hstep){

//    printf("x: %d, count: %d\n", x, verticalPointCount[c_FIELD_GREEN][x]);


    int edgeY = iparams_.height-1;    // This will mark the boundary for this x coordinate

/*    // We will assume the first run is ok.
    if (verticalPointCount[c_FIELD_GREEN][x] > 0){
      edgeY = verticalPoint[c_FIELD_GREEN[x][verticalPointCount[c_FIELD_GREEN][x] - 1]].yi;
    }
*/


    // Position edgeYPos = cmatrix_.getWorldPosition(x, edgeY);
    
    Position left = positions[12][(int)floor(1.0 * x / xjump)];
    Position right = positions[12][(int)ceil(1.0 * x / xjump)];
 
    Position edgeYPos = left + ((right - left) * (fmod(1.0 * x, xjump) / xjump ));

//    std::cout << left << " " << right << " " << fmod(1.0 * x, xjump) / xjump << "\n";


//    Position edgeYPos = (left + right) / 2.0;



    // Iterate over the vertical run
    for (int i = verticalPointCount[c_FIELD_GREEN][x] - 1; i >= 0; i--){


      // Compare yf of this run to the current edgeY (i.e. yi of previous run)
      // Position yfPos = cmatrix_.getWorldPosition(x, verticalPoint[c_FIELD_GREEN][x][i].yf);  


      int yf = verticalPoint[c_FIELD_GREEN][x][i].yf;

      Position topLeft = positions[(int)floor(1.0 * yf / yjump)][(int)floor(1.0 * x / xjump)];
      Position topRight = positions[(int)floor(1.0 * yf / yjump)][(int)ceil(1.0 * x / xjump)];
      Position bottomLeft = positions[(int)ceil(1.0 * yf / yjump)][(int)floor(1.0 * x / xjump)];
      Position bottomRight = positions[(int)ceil(1.0 * yf / yjump)][(int)ceil(1.0 * x / xjump)];


      Position yInterp = bottomLeft  + ((topLeft - bottomLeft) * (1 - (fmod(1.0 * yf , yjump) / yjump)));
      Position xInterp = bottomLeft + ((bottomRight - bottomLeft) * (fmod(1.0 * x, xjump) / xjump));
      Position yfPos(yInterp.x, xInterp.y, yInterp.z);


//      Position yfPos = (topLeft + topRight + bottomLeft + bottomRight) / 4.0;


//      cout << yfPos << " " << yfPosNew << "\n";

      // If the distance is less than 1m projected, set the new edgeY to be 
      
      double dist = sqrt(((edgeYPos.x - yfPos.x) * (edgeYPos.x - yfPos.x)) + ((edgeYPos.y - yfPos.y) * (edgeYPos.y - yfPos.y))); 


//      cout << dist << "\n";

//      double dist = edgeY - verticalPoint[c_FIELD_GREEN][x][i].yf;
      if (dist < 500){
        edgeY = verticalPoint[c_FIELD_GREEN][x][i].yi;
        // edgeYPos = cmatrix_.getWorldPosition(x, edgeY);
      
      
        Position topLeftYi = positions[(int)floor(1.0 * edgeY / yjump)][(int)floor(1.0 * x / xjump)];
        Position topRightYi = positions[(int)floor(1.0 * edgeY / yjump)][(int)ceil(1.0 * x / xjump)];
        Position bottomLeftYi = positions[(int)ceil(1.0 * edgeY / yjump)][(int)floor(1.0 * x / xjump)];
        Position bottomRightYi = positions[(int)ceil(1.0 * edgeY / yjump)][(int)ceil(1.0 * x / xjump)];


        Position yInterpYi = bottomLeftYi  + ((topLeftYi - bottomLeftYi) * (1 - (fmod(1.0 * edgeY, yjump) / yjump)));
        Position xInterpYi = bottomLeftYi + ((bottomRightYi - bottomLeftYi) * (fmod(1.0 * x, xjump) / xjump));
        
        //Position edgeYPos(yInterpYi.x, xInterpYi.y, yInterpYi.z);
        edgeYPos.x = yInterpYi.x;
        edgeYPos.y = xInterpYi.y;



//        edgeYPos = (topLeftYi + topRightYi + bottomLeftYi + bottomRightYi) / 4.0;
      
      
      
      }


//      printf("yi %d yf %d dy %d\n", verticalPoint[c_FIELD_GREEN][x][i].yi, verticalPoint[c_FIELD_GREEN][x][i].yf, verticalPoint[c_FIELD_GREEN][x][i].dy);
      

    }

    // Set the field edge point candidate to be edgeY
    fieldEdgePoints[index++] = edgeY;

  }




  VisionTimer::Stop("FieldEdgeDetector(%s)::iterateX", camera_);


  //printf("Finished extracting edge points\n");

  //printf("Setting up hullPointCands, index %d\n", index);


  
  hullPointCands.clear();

  //printf("About to add first element. Iparams(%d,%d)\n", iparams_.width / 2, iparams_.height);


  //printf("Populating point array\n");

  // Throw them into an array of points (in reverse order) and find convex hull
  for (int i = 0; i < index; i++){


    int x = ((index - 1 - i) * hstep);
    
//    printf("Adding (%d, %d)\n", x, fieldEdgePoints[index - 1 - i]);
    
    hullPointCands.push_back(FieldEdgePoint(x, fieldEdgePoints[index - 1 - i]));
  }




  //printf("Calculating convex hull points\n");

//  convexHull(hullPointCands, index);
  // Score this hull somehow.
  // Find potential outliers in the hull
  // Remove them, form a new hull, and rescore
  // Shorter/hacky version
//  do {


  VisionTimer::Start(80, "FieldEdgeDetector(%s)::removeOutliers", camera_);

    removeOutliers();
    
  VisionTimer::Stop("FieldEdgeDetector(%s)::removeOutliers", camera_);
    
  VisionTimer::Start(80, "FieldEdgeDetector(%s)::cvxHull", camera_);
    convexHull();

  VisionTimer::Stop("FieldEdgeDetector(%s)::cvxHull", camera_);
    
    //  } while (removeOutliers(hullPointCands));




  // Find all the points that are below the hull

  VisionTimer::Start(80, "FieldEdgeDetector(%s)::belowHull", camera_);
  int greenIndex = hullPointCands.size()-1;
  int BELOW_THRESH = 16; 

  hullSegments.clear();

  for (int i = 0; i < hullPoints.size()-1; i++){


    // Fit a line
    LineSegment line(hullPoints[i], hullPoints[i+1]);
    hullSegments.push_back(line);

//    printf("Forming line from (%f,%f) to (%f,%f)\n", hullPoints[i].x, hullPoints[i].y, hullPoints[i+1].x, hullPoints[i+1].y);
    
    //
    while (hullPointCands[greenIndex].x <= hullPoints[i+1].x && greenIndex >= 0){

//      printf("Checking point at (%f,%f)\n", hullPointCands[greenIndex].x, hullPointCands[greenIndex].y);

      // Extrapolate the point
      float Y = line.getYGivenX(hullPointCands[greenIndex].x);

      // See if it's below
      if (hullPointCands[greenIndex].y > Y + BELOW_THRESH){
        hullPointCands[greenIndex].below = true;
      }

      hullPointCands[greenIndex].hullY = Y;

      greenIndex--;
    }

  }


  VisionTimer::Stop("FieldEdgeDetector(%s)::belowHull", camera_);




  //printf("Completed successfully.\n");


//  circle(m_img, cv::Point(x, color_segmenter_.fieldEdgePoints[x/hstep]), 2, Scalar(0, 200, 0), 2);



  VisionTimer::Stop("FieldEdgeDetector(%s)::findFieldEdgePoints", camera_);

  return true;
}

bool FieldEdgeDetector::continueObjectRun(FieldEdgePoint &p, int lastY){

  const int Y_THRESH = 5;

  // Check the point is below
  if (!p.below) return false;

  // Check Y proximity 
  if (abs(p.y - lastY) <= Y_THRESH){
    return true;
  }

  return false;
}


void FieldEdgeDetector::filterObjects(){

  const float ROBOT_LEG_HEIGHT = 200;     // 300 is right below jersey. Most of legs are below 200
 
//  printf("=> Extracting leg position\n");

  for (int i = 0; i < objects.size(); i++){

    Position objectBase = cmatrix_.getWorldPosition(objects[i].avgX, objects[i].yf);

    // Find where robot's legs end approximately.
    Coordinates legCoord = cmatrix_.getImageCoordinates(objectBase.x, objectBase.y, ROBOT_LEG_HEIGHT);

    objects[i].yi = legCoord.y;
    objects[i].dy = objects[i].yf - objects[i].yi + 1;
    objects[i].avgY = objects[i].yi + (objects[i].dy / 2.0);

  }

//  printf("Removing non-white objects\n");
  // Remove non-white objects. These are most likely people legs, etc.
  const float WHITE_THRESHOLD = 0.3;
  for (int i = 0; i < objects.size(); i++){

    if (!objects[i].valid) continue;

    int whitePixelCount = 0;
    int total = 0;

    int xmin = objects[i].xi;
    int ymin = objects[i].yi;
    int xmax = objects[i].xf;
    int ymax = objects[i].yf;

    ymin -= ymin % vstep;
    ymax += vstep - ymax % vstep;
    xmin -= xmin % hstep;
    xmax += hstep - xmax % hstep;

    xmin = std::max(xmin, 0);
    xmax = std::min(xmax, iparams_.width - 1);
    ymin = std::max(ymin, 0);
    ymax = std::min(ymax, iparams_.height - 1);



    for (int x = xmin; x <= xmax; x+=hstep){
      for (int y = ymin; y <= ymax; y+=vstep){
        
        int c = getSegPixelValueAt(x,y);
        if (c == c_WHITE || c == c_ROBOT_WHITE){
          whitePixelCount++;
        }
        total++;
      }
    }

    float pct = (float)(whitePixelCount) / total;
    if (!total) pct = 0;

//    printf("Object at (%d,%d) pct was %d / %d = %f\n", objects[i].avgX, objects[i].avgY, whitePixelCount, total, pct);

    if (pct < WHITE_THRESHOLD)
      objects[i].valid = false;
  }


  // Remove objects that are too close to the field boundary. These tend to be artifacts of the field edge boundary detection. 
//  printf("Removing objects on boundary\n");
  for (int i = 0; i < objects.size(); i++){

    if (!objects[i].valid) continue;

      Point2D minClosestPoint;

      // Technically we should compute distance to closest hull point, not just directly vertical.
      // And this should be distance in projected space not image space. 
      float minDistance = 10000;
      for (int j = 0; j < hullSegments.size(); j++){
        Point2D closestHullPoint = hullSegments[j].getPointOnSegmentClosestTo(Point2D(objects[i].avgX, objects[i].yf));


        Position projHullPoint = cmatrix_.getWorldPosition(closestHullPoint.x, closestHullPoint.y);
        Position projBasePoint = cmatrix_.getWorldPosition(objects[i].avgX, objects[i].yf);
        
        objects[i].pos = projBasePoint;

        float distance = Point2D(projHullPoint.x, projHullPoint.y).getDistanceTo(Point2D(projBasePoint.x, projBasePoint.y));

//        printf("Hull segment from (%f,%f) to (%f,%f)\n", hullSegments[j].start.x, hullSegments[j].start.y, hullSegments[j].end.x, hullSegments[j].end.y);

//        printf("Closest hull point (%f,%f) distance %f\n", closestHullPoint.x, closestHullPoint.y, distance);
       
        if (distance < minDistance){
          minDistance = distance;
          minClosestPoint = closestHullPoint;
        }


      }

//    printf("Object at (%d,%d) dist was %f to (%3.0f,%3.0f)\n", objects[i].avgX, objects[i].avgY, minDistance, minClosestPoint.x, minClosestPoint.y);
    
//      Position projHullPoint = cmatrix_.getWorldPosition(minClosestPoint.x, minClosestPoint.y);
//      printf("  Proj hull: (%f,%f) proj base (%f,%f)\n", projHullPoint.x, projHullPoint.y, objects[i].pos.x, objects[i].pos.y);





      if (minDistance < BORDER_STRIP_WIDTH * 0.75){
        objects[i].valid = false;
      }
      objects[i].distToEdge = minDistance;

  }




  // Merge objects that are close together
//  printf("Merging close objects\n");
  const float OBJECT_MERGE_DIST = 200;

  for (int i = 0; i < static_cast<int>(objects.size())-1; i++){

    if (!objects[i].valid) continue;

//    printf("I %d / %d\n", i, objects.size()-1);

    for (int j = i+1; j < objects.size(); j++){

//      printf("  J %d / %d\n", j, objects.size());

      if (!objects[j].valid) continue;

      float dist = (objects[i].pos - objects[j].pos).abs();

//      printf("Dist between (%d,%d) and (%d,%d) is %f\n", objects[i].avgX, objects[i].avgY, objects[j].avgX, objects[j].avgY, dist);

      // Assumes objects sorted by x direction
      if (dist < OBJECT_MERGE_DIST){

        objects[i].xf = objects[j].xf;
        objects[i].dx = objects[i].xf - objects[i].xi + 1;
        objects[i].avgX = (objects[i].xi + objects[i].xf) / 2.0;
        if (objects[j].yi < objects[i].yi){
          objects[i].yi = objects[j].yi;
          objects[i].dy = objects[i].yf - objects[i].yi + 1;
        }
        if (objects[j].yf > objects[i].yf){
          objects[i].yf = objects[j].yf;
          objects[i].dy = objects[i].yf - objects[i].yi + 1;
        }
        objects[i].avgY = (objects[i].yi + objects[i].yf) / 2.0;

        // Update pos
        objects[i].pos = cmatrix_.getWorldPosition(objects[i].avgX, objects[i].yf);

        // TODO: recompute the correct distance to edge
        objects[i].distToEdge = max(objects[i].distToEdge, objects[j].distToEdge);

        objects[j].valid = false;
      }

    }

  }


}


void FieldEdgeDetector::findPossibleObjects(){

  const int MIN_SEGMENT_SIZE = 4;
  objects.clear();
  
  // Iterate over hullPointCands
 
  int xi = 0;
  int xiHullY = 0;
  int lastY = 0;
  int minY = 0;       // This is more like bottom y which is maxY

  bool objectRun = false;

  for (int i = hullPointCands.size()-1; i >= 0; i--){


    // We can either end the run if one pixel misses or if 2 pixels miss... 
    if (objectRun && !continueObjectRun(hullPointCands[i], lastY)){

      if ((hullPointCands[i].x - xi) / hstep >= MIN_SEGMENT_SIZE){
        
        // Create a new object point
        VisionObjectCandidate lp;
        lp.xi = xi;
        lp.xf = hullPointCands[i].x - hstep;
        lp.dx = lp.xf - lp.xi + 1;
        lp.avgX = lp.xi + (lp.dx / 2.0);
        lp.yf = minY;
        lp.yi = (xiHullY + hullPointCands[i].hullY) / 2.0;
        lp.hullY = lp.yi;
        lp.dy = lp.yf - lp.yi + 1;
        lp.avgY = lp.yi + (lp.dy / 2.0);
        lp.valid = true;
        lp.robotCandidate = false;
        lp.postCandidate = false;
        objects.push_back(lp);
      }

      objectRun = false;
    }
    else if (objectRun && continueObjectRun(hullPointCands[i], lastY)){
      lastY = hullPointCands[i].y;
      minY = max(minY, lastY);
    }

    // Start a new object run
    if (!objectRun && hullPointCands[i].below){
      objectRun = true;
      xi = hullPointCands[i].x;
      xiHullY = hullPointCands[i].hullY;
      lastY = hullPointCands[i].y;
      minY = lastY;
    }



  }

  // Create segments


  // Draw box

//  printf("Found %d objects\n", objects.size());

}




// This is a really crude way of doing it. Need something better
bool FieldEdgeDetector::removeOutliers(){

  const int YTHRESH = 12;

  bool foundOutlier = false;

//  printf("Hull point cands size: %d\n", hullPointCands.size());

  // Loop through the convex hull points
  for (int i = 0; i < hullPointCands.size(); i++){

    // Check if the two adjacent points in hullPointCands are both below, and by a certain threshold
  
//    int index = hullPointCands.size() - 1 - (hullPoints[i].x / hstep);


    int adj1 = max(0, i-1);
    int adj2 = min(i+1, static_cast<int>(hullPointCands.size())-1);
    int adj3 = max(0, i-2);
    int adj4 = min(i+2, static_cast<int>(hullPointCands.size())-1);

    if (hullPointCands[adj1].y > hullPointCands[i].y + YTHRESH || hullPointCands[adj2].y > hullPointCands[i].y + YTHRESH  || hullPointCands[adj3].y > hullPointCands[i].y + YTHRESH || hullPointCands[adj4].y > hullPointCands[i].y + YTHRESH){

      // If so, mark this hullPointCand point as not valid
      hullPointCands[i].valid = false;    
      //printf("Eliminating hull point at (%d,%d)\n", hullPointCands[index].x, hullPointCands[index].y);
      foundOutlier = true;
    }


  }

//  printf("Finished removing outliers\n");

  return foundOutlier;

}







// Prints convex hull of a set of n points.
void FieldEdgeDetector::convexHull()
{

  // Create an empty stack and push first three points
  // to it.

//  printf("Entering convex hull\n");

//  if (n < 2) return;
  
  hullPoints.clear();
  stack<FieldEdgePoint> S;

  if (hullPointCands.size() < 2) return;

  int i = 0;
  while(S.size() < 2 && i < hullPointCands.size()){
    if (hullPointCands[i].valid){
      S.push(hullPointCands[i]);
    }
    i++;
  }

  if (S.size() < 2) return;

//  printf("Initialized 2 hull points\n");


//  printf("Processing remaining points\n");

  // Process remaining n-3 points
  for (; i < hullPointCands.size(); i++)
  {
    
    if (!hullPointCands[i].valid)
      continue;

//    printf("Iteration %d\n", i);
    // Keep removing top while the angle formed by
    // points next-to-top, top, and points[i] makes
    // a non-left turn
    while (S.size() > 2 && orientation(nextToTop(S), S.top(), hullPointCands[i]) != 1){
      S.pop();
//      printf("Popped.\n");
    }
//    printf("Pushed\n");
    S.push(hullPointCands[i]);
  }

//  printf("Popping stack\n");
  // Now stack has the output points, print contents of stack
  
//  printf("Completed forming hull\n");

  
  while (!S.empty())
  {
    FieldEdgePoint p = S.top();
    hullPoints.push_back(p);
//    cout << "(" << p.x << ", " << p.y <<")" << endl;
    S.pop();
  }

//  printf("Finished popping points\n");

}





// A utility function to find next to top in a stack
FieldEdgePoint FieldEdgeDetector::nextToTop(stack<FieldEdgePoint> &S)
{
  FieldEdgePoint p = S.top();
  S.pop();
  FieldEdgePoint res = S.top();
  S.push(p);
  return res;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int FieldEdgeDetector::orientation(FieldEdgePoint p, FieldEdgePoint q, FieldEdgePoint r)
{
  int val = (q.y - p.y) * (r.x - q.x) -
    (q.x - p.x) * (r.y - q.y);

//  printf("Val: %d\n", val);

  if (val == 0) return 0;  // colinear
  return (val > 0)? 1: 2; // clock or counterclock wise
}



/** Tries to find a line using RANSAC using field edge point candidates in the specified range */
bool FieldEdgeDetector::ransacLine(int start, int end, Line2D &bestLine){

  const int MAX_ITERS = 100;
  const float CONSENSUS_THRESH = 0.6 * (end - start + 1);
  float best_consensus = -1;
  const int VOTE_THRESH = 2 * vstep;

  int pt1, pt2;
  printf("range is %d,%d\n",start,end);
  for (int i = 0; i < MAX_ITERS; i++){
    int pt1X = (rand() % (end-start+1)) + start;
    int pt2X = (rand() % (end-start+1)) + start;
    /*printf("sampled x coordinates are %d,%d\n",pt2X*hstep,pt1X*hstep);*/

    if (pt1X == pt2X)
      continue;
    // Reproject
    //pt1X *= hstep;
    //pt2X *= hstep;
    Line2D line = Line2D::makeLineFromTwoPoints(Point2D(pt1X*hstep, color_segmenter_.fieldEdgePoints[pt1X]), Point2D(pt2X*hstep, color_segmenter_.fieldEdgePoints[pt2X]));
    int consensus = 0;
    for (int p = start; p <= end; p++){
      if (line.getDistanceToPoint(Point2D(p*hstep, color_segmenter_.fieldEdgePoints[p])) <= VOTE_THRESH){
        consensus++;
      }
    }
    if (consensus > best_consensus){
      bestLine.m_a = line.m_a;
      bestLine.m_b = line.m_b;
      bestLine.m_c = line.m_c;
      best_consensus = consensus;
      pt1 = pt1X;
      pt2 = pt2X;
      //return true;
    }

  }
  
  if (best_consensus > CONSENSUS_THRESH) {
/*    circle(m_img, cv::Point(pt1*hstep, color_segmenter_.fieldEdgePoints[pt1]), 3, Scalar(255,0, 0), 3);
    circle(m_img, cv::Point(pt2*hstep, color_segmenter_.fieldEdgePoints[pt2]), 3, Scalar(255,0, 0), 3);
    Line2D lineObj = Line2D::makeLineFromTwoPoints(Point2D(pt1*hstep, color_segmenter_.fieldEdgePoints[pt1]), Point2D(pt2*hstep, color_segmenter_.fieldEdgePoints[pt2]));


    line(m_img, cv::Point(pt1*hstep, color_segmenter_.fieldEdgePoints[pt1]), cv::Point(pt2*hstep, color_segmenter_.fieldEdgePoints[pt2]), Scalar(255, 255, 255), 5);
*/
    return true;
  } else {
    return false;
  }
}



/** Need to decide whether */




// DO NOT REMOVE and leave at bottom
// vim: expandtab:noai:sts=2:sw=2:ts=2
