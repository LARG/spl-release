#include "FieldEdgeDetector.h"

using namespace cv;

FieldEdgeDetector::FieldEdgeDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter, BlobDetector& blob_detector) : DETECTOR_INITIALIZE, color_segmenter_(segmenter), blob_detector_(blob_detector) {


  hstep = (1 << iparams_.defaultHorizontalStepScale);
  vstep = (1 << iparams_.defaultVerticalStepScale);

  fieldEdgePoints = new int[iparams_.width / hstep];

  srand (time(NULL));

}

/** Main function that detects field edges */
bool FieldEdgeDetector::detectFieldEdges(){



  m_img = color::rawToMat(vblocks_.image->getImgTop(), iparams_);

  findFieldEdgePoints();
  
  // Fit lines on left and right 
  Line2D left;
  
  if (ransacLine(0, (4* iparams_.width) /(10 * hstep), left)){
    line(m_img, cv::Point(0, left.getYGivenX(0)), cv::Point(iparams_.width - 1, left.getYGivenX(iparams_.width - 1)), Scalar(0, 255, 0), 2);
  }


  Line2D right;
  if (ransacLine( (6 * iparams_.width) / (10 * hstep), (iparams_.width / hstep) - 1, right)){
    line(m_img, cv::Point(0, right.getYGivenX(0)), cv::Point(iparams_.width - 1, right.getYGivenX(iparams_.width - 1)), Scalar(0, 0, 255), 2);
  }

  imwrite("/home/eladlieb/Desktop/fieldEdge.png", m_img);
  return true;
}


/** Finds the top-most green pixel using green vertical runs, and places in fieldEdgePoints as a possible field edge point candidate */
bool FieldEdgeDetector::findFieldEdgePoints(){
  const int MIN_RUN_LENGTH = 5;
  for (int x = 0; x < iparams_.width; x += hstep){
//    printf("Col %d green run count %d\n", x, color_segmenter_.vGreenPosition[x]);
    
//    for (int r = 0; r < color_segmenter_.verticalPointCount[c_FIELD_GREEN][x]; r++){
//      VisionPoint *lp = &color_segmenter_.verticalPoint[c_FIELD_GREEN][x][r];

      
    circle(m_img, cv::Point(x, color_segmenter_.fieldEdgePoints[x/hstep]), 2, Scalar(0, 200, 0), 2);
      
      
//      if ((lp->dy / vstep) >= MIN_RUN_LENGTH){
//        fieldEdgePoints[x/hstep] = lp->yi;
//        break;
//      }
//    }
  }





  return true;
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
    circle(m_img, cv::Point(pt1*hstep, color_segmenter_.fieldEdgePoints[pt1]), 3, Scalar(255,0, 0), 3);
    circle(m_img, cv::Point(pt2*hstep, color_segmenter_.fieldEdgePoints[pt2]), 3, Scalar(255,0, 0), 3);
    Line2D lineObj = Line2D::makeLineFromTwoPoints(Point2D(pt1*hstep, color_segmenter_.fieldEdgePoints[pt1]), Point2D(pt2*hstep, color_segmenter_.fieldEdgePoints[pt2]));


    line(m_img, cv::Point(pt1*hstep, color_segmenter_.fieldEdgePoints[pt1]), cv::Point(pt2*hstep, color_segmenter_.fieldEdgePoints[pt2]), Scalar(255, 255, 255), 5);

    return true;
  } else {
    return false;
  }
}



/** Need to decide whether */




// DO NOT REMOVE and leave at bottom
// vim: expandtab:noai:sts=2:sw=2:ts=2
