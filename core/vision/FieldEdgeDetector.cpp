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
  findFieldEdgePoints();
  

  return true;
}


/** Finds the top-most green pixel using green vertical runs, and places in fieldEdgePoints as a possible field edge point candidate */
bool FieldEdgeDetector::findFieldEdgePoints(){
 


  VisionPoint ***verticalPoint = color_segmenter_.verticalPoint;
  uint32_t **verticalPointCount = color_segmenter_.verticalPointCount;

  int index = 0;

  // Iterate over X
  for (int x = 0; x < iparams_.width; x += hstep){



    int edgeY = iparams_.height-1;    // This will mark the boundary for this x coordinate
    Position edgeYPos = cmatrix_.getWorldPosition(x, edgeY);

    // Iterate over the vertical run
    for (int i = verticalPointCount[c_FIELD_GREEN][x] - 1; i >= 0; i--){


      // Compare yf of this run to the current edgeY (i.e. yi of previous run)
      Position yfPos = cmatrix_.getWorldPosition(x, verticalPoint[c_FIELD_GREEN][x][i].yf);  

      // If the distance is less than 1m projected, set the new edgeY to be 
      
      double dist = sqrt(((edgeYPos.x - yfPos.x) * (edgeYPos.x - yfPos.x)) + ((edgeYPos.y - yfPos.y) * (edgeYPos.y - yfPos.y))); 

      if (dist < 500){
        edgeY = verticalPoint[c_FIELD_GREEN][x][i].yi;
        edgeYPos = cmatrix_.getWorldPosition(x, edgeY);
      }

    }

    // Set the field edge point candidate to be edgeY
    fieldEdgePoints[index++] = edgeY;

  }

  hullPointCands.clear();

  // Throw them into an array of points (in reverse order) and find convex hull
  for (int i = 0; i < index; i++){
    int x = ((index - 1 - i) * hstep);
    hullPointCands.push_back(FieldEdgePoint(x, fieldEdgePoints[index - 1 - i]));
  }


    removeOutliers();
    convexHull();




  // Find all the points that are below the hull

  int greenIndex = hullPointCands.size()-1;
  int BELOW_THRESH = 16; 

  hullSegments.clear();

  for (int i = 0; i < hullPoints.size()-1; i++){


    // Fit a line
    LineSegment line(hullPoints[i], hullPoints[i+1]);
    hullSegments.push_back(line);

    while (hullPointCands[greenIndex].x <= hullPoints[i+1].x && greenIndex >= 0){

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
      
  return true;
}





// This is a really crude way of doing it. Need something better
bool FieldEdgeDetector::removeOutliers(){

  const int YTHRESH = 12;

  bool foundOutlier = false;

  // Loop through the convex hull points
  for (int i = 0; i < hullPointCands.size(); i++){

    int adj1 = max(0, i-1);
    int adj2 = min(i+1, static_cast<int>(hullPointCands.size())-1);
    int adj3 = max(0, i-2);
    int adj4 = min(i+2, static_cast<int>(hullPointCands.size())-1);

    if (hullPointCands[adj1].y > hullPointCands[i].y + YTHRESH || hullPointCands[adj2].y > hullPointCands[i].y + YTHRESH  || hullPointCands[adj3].y > hullPointCands[i].y + YTHRESH || hullPointCands[adj4].y > hullPointCands[i].y + YTHRESH){

      // If so, mark this hullPointCand point as not valid
      hullPointCands[i].valid = false;    
      foundOutlier = true;
    }


  }

  return foundOutlier;

}


// Prints convex hull of a set of n points.
void FieldEdgeDetector::convexHull()
{

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

  // Process remaining n-3 points
  for (; i < hullPointCands.size(); i++)
  {
    
    if (!hullPointCands[i].valid)
      continue;

    while (S.size() > 2 && orientation(nextToTop(S), S.top(), hullPointCands[i]) != 1){
      S.pop();
    }
    S.push(hullPointCands[i]);
  }

  while (!S.empty())
  {
    FieldEdgePoint p = S.top();
    hullPoints.push_back(p);
    S.pop();
  }

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

// DO NOT REMOVE and leave at bottom
// vim: expandtab:noai:sts=2:sw=2:ts=2
