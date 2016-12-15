#include <vision/HoughDetector.h>
#include <vision/ColorSegmenter.h>

using namespace std;
using namespace cv;


HoughDetector::HoughDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter) : DETECTOR_INITIALIZE, color_segmenter_(segmenter) {

  setImagePointers();
  
  // TODO: Read these parameters from ImageParams
  //int hstep, vstep;
//  color_segmenter_.getStepSize(XSTEP, YSTEP);
  XSTEP = 2;
  YSTEP = 2;    // 4 is the same as default vertical step scale
  
  // 2 => 16Hz, 8 => 21Hz

  vertLines.reserve(320);
  horizonSet = false;
}

HoughDetector::~HoughDetector() {

}

bool HoughDetector::setImagePointers() {
  if(vblocks_.image == NULL) {
    printf("No image block loaded! Hough line detection failed.\n");
    return false;
  }
  if(camera_ == Camera::TOP) {
    img_ = vblocks_.image->getImgTop();
    if(!img_) return false;
  }
  else {
    img_ = vblocks_.image->getImgBottom();
    if(!img_) return false;
  }
  return true;
}

bool HoughDetector::setHorizon(HorizonLine horizon) {
  horizonSet = true;
  horizon_ = horizon;
  horizonY = horizon.offset + (horizon.gradient * iparams_.width / 2);
  int slack = 0.10 * (iparams_.height - horizonY);
  horizonY -= slack;
  horizonY -= (horizonY % YSTEP);
  if (horizonY < 0){
    horizonY = 0;
  }
  // The horizon is below the image. We probably fell down.
  if (horizonY >= iparams_.height){
    horizonY = iparams_.height - 1;
    horizonSet = false;
  }

// TODO: Since we are extracting the whole image, for now leave this off. 
//  horizonY = 0;

  return horizonSet;
}



bool HoughDetector::extractLines(){

  
  vertLines.clear();

  //int id = tic();

  if (!setImagePointers())
    return false;

  //printtime("Setting image pointers", id);

  // Extract subsampled grayscale image
  //id = tic();

  VisionTimer::Start("Extracting MAT");
  height = min(iparams_.height - horizonY, iparams_.height / 10);


//  height = iparams_.height - horizonY;

  m_img = Mat(height / YSTEP, iparams_.width / XSTEP, CV_8U);
  extractMat(m_img);
  VisionTimer::Stop("Extracting MAT");
  
/*  
  Mat raw_img = color_segmenter_.m_img;
  Rect roi(0, horizonY / YSTEP, raw_img.cols, raw_img.rows - (horizonY / YSTEP));
  Mat m_img = raw_img(roi);
*/

  //printtime("Extracting MAT", id);

  // Use Sobel/Schaar/Canny edge detection to find edges
  //id = tic();
  VisionTimer::Start("Edge Detection");
  edges = Mat();
  findEdges(m_img, edges, 100); // 75  // 60     // 75
  VisionTimer::Stop("Edge Detection");
  //printtime("Schaar edge detection", id);

  // Hough transform to find lines
  //id = tic();
  VisionTimer::Start("Hough Lines");
  lines.clear();
  detectLines(edges, lines);
  VisionTimer::Stop("Hough Lines");
  //printtime("Hough line detection", id);

  // Filtering and population of postCandidates object
  //id = tic();
  
  VisionTimer::Start("Line Filtering / Reprojection");
//  vector<Blob> lineCandidates;
  filterLines(lines, vertLines);
  //printtime("Line filtering", id);



  // Fix blob data to be in original coordinate system
  //id = tic();
  for (int i = 0; i < vertLines.size(); i++){
    Blob *b = &vertLines[i];
    // Reproject to original coordinate system
    b->xi = (b->xi * XSTEP);
    b->yi = horizonY + (b->yi * YSTEP);
    b->xf = b->xf * XSTEP;
    b->yf = horizonY + (b->yf * YSTEP);
    b->avgX = (b->xi + b->xf) / 2;
    b->avgY = (b->yi + b->yf) / 2;
  }
  //printtime("Reprojection", id);

/*

  // Prefilter to remove blobs that are part of field lines by checking for green above them
  // This is necessary, otherwise when we scan adjacent lines to form posts, the field lines will get in the way, preventing the post from being formed. 

  //id = tic();
  for (int i = 0; i < lineCandidates.size(); i++){

    Blob b = lineCandidates[i];

//    if (b.yi < 20 || color_segmenter_.colorPercentageInBox(b.xi, b.xf, b.yi - 10, b.yi, c_FIELD_GREEN) < 0.1){
      postCandidates.push_back(lineCandidates[i]);
//    }

  }
*/  
  
  //printtime("Post-filtering", id);



  VisionTimer::Stop("Line Filtering / Reprojection");


  return true;

}

void HoughDetector::detectLines(Mat &edges, vector<Vec4i> &lines){
  int votesNeeded = 20;    // 20      //50;         // 30
  double minLineLength = 20;     //20; // 10    //40;    // 30
  double maxLineGap = 5; // 5

  double resolution = 1;    // 5

//  time = clock();
  HoughLinesP(edges, lines, resolution, CV_PI/1, votesNeeded, minLineLength, maxLineGap);  
//  time = clock() - time;
//  printf("Hough line detection: %f sec\n", ((float)time)/CLOCKS_PER_SEC);

  // Sort the lines by x coordinate

}

// This merges lines that are close to each other, etc.
// ASSUMES ONLY VERTICAL LINES
void HoughDetector::filterLines(vector<Vec4i> &lines, vector<Blob> &postCandidates){


  std::sort(lines.begin(), lines.end(), vec4iSortByX());

/*  Blob blob;
  blob.xi = lines[0][0];
  blob.yi = lines[0][3];
  blob.xf = lines[0][0];
  blob.yf = lines[0][1];
*/
  // Merge and form blobs/line candidates



  VisionTimer::Start("Filtering (merging)");
  int yMergeThresh = 0;
  int xMergeThresh = 5; // 5;    // 10

  for (size_t i = 0; i < lines.size(); i++){

    Vec4i line = lines[i];

    int lineSize;
    int lineStrength;

//  VisionTimer::Start("Filtering (edge data)");
    getEdgeData(line, lineSize, lineStrength);
//    printf("Line size: %d strength: %d\n", lineSize, lineStrength);

//  VisionTimer::Stop("Filtering (edge data)");



//  VisionTimer::Start("Filtering (push backs)");
    
  

    // Merge with last blob
    // Assumes lines are x sorted

// Note that if you wanted to merge into a single blob, you would use .xf. However, this tends to merge things that are close to the post too. You can get better 'blobs' by changing xMergeThresh to 1 in addition to changing this to .xf. Otherwise set to xi and increase xmergethresh to like 5.  
    if (i > 0 && line[0] - postCandidates.back().xi <= xMergeThresh ){
      
      // Verify y overlap
//      if (!(line[3] - postCandidates.back().yf > yMergeThresh || postCandidates.back().yi - line[1] > yMergeThresh)){
      
        Blob *lastBlob = &postCandidates.back();
        lastBlob->xf = line[0];
        if (line[3] < lastBlob->yi){
          lastBlob->yi = line[3];
        }
        if (line[1] > lastBlob->yf){
          lastBlob->yf = line[1];
        }
        lastBlob->edgeSize += lineSize;
        lastBlob->edgeStrength += lineStrength;
//     }
/*      // Only merge if there is no green between
      else {
  
        int xmin = XSTEP * std::min(line[0], static_cast<int>(postCandidates.back().xi));
        int xmax = XSTEP * std::max(line[0], static_cast<int>(postCandidates.back().xf));
        int ymin = horizonY + YSTEP * std::min(line[3], static_cast<int>(postCandidates.back().yi));
        int ymax = horizonY + YSTEP * std::max(line[1], static_cast<int>(postCandidates.back().yf));

        float greenBetween = color_segmenter_.colorPercentageInBox(xmin, xmax, ymin, ymax, c_FIELD_GREEN);
#ifdef TOOL
        printf("Trying to merge (%d,%d) and (%d,%d): greenbetween (%d,%d) (%d,%d)  %f\n", line[0]*XSTEP, line[3]*YSTEP + horizonY, postCandidates.back().xi*XSTEP, postCandidates.back().yf*YSTEP+horizonY, xmin, ymin, xmax, ymax, greenBetween);
#endif          

        // And not toooo far away
        if (greenBetween < 0.25){
          // Merge
          Blob *lastBlob = &postCandidates.back();
          lastBlob->xf = line[0];
          if (line[3] < lastBlob->yi){
            lastBlob->yi = line[3];
          }
          if (line[1] > lastBlob->yf){
            lastBlob->yf = line[1];
          }
          lastBlob->edgeSize += lineSize;
          lastBlob->edgeStrength += lineStrength;

        }
        else {
          // New
          
          blob.xi = line[0];
          blob.yi = line[3];
          blob.xf = line[0];
          blob.yf = line[1];
          blob.edgeSize = lineSize;
          blob.edgeStrength = lineStrength;
         postCandidates.push_back(blob);
        }
      
      }
*/

    }
    // Push back and create new blob
    else {
      Blob blob;
      blob.xi = line[0];
      blob.yi = line[3];
      blob.xf = line[0];
      blob.yf = line[1];
      blob.edgeSize = lineSize;
      blob.edgeStrength = lineStrength;
      vertLines.push_back(blob);
    }


//  VisionTimer::Stop("Filtering (push backs)");




  }




  VisionTimer::Stop("Filtering (merging)");


    
    /*
    int whiteCount = 0;
    for (int x = b->xi; x <= b->xf; x++){
      for (int y = b->yi; y <= b->yf; y++){
        int c = getSegPixelValueAt(x, y);
        //int c = ColorTableMethods::xy2color(img_, x, y, iparams_.width)
        
        if (c == c_WHITE || c == c_ROBOT_WHITE)
          whiteCount++;
      }
    }
    b->whiteCount = whiteCount;
*/
//    printf("White count: %d\n", b->whiteCount);
//    printf("Size: %d\n", (b->xf - b->xi + 1) * (b->yf - b->yi + 1));
//    printf("White %: %f\n", static_cast<float>(b->whiteCount) / static_cast<float>((b->xf - b->xi + 1) * (b->yf - b->yi + 1)));
    

//      Position pos = cmatrix_.getWorldPosition(b->xf, b->yf);
//      printf("Blob %d: pos(%f, %f, %f) Dist: %f White%: %f\n", i, pos.x, pos.y, pos.z, cmatrix_.groundDistance(pos), static_cast<float>(b->whiteCount * hstep * vstep) / static_cast<float>((b->xf - b->xi + 1) * (b->yf - b->yi + 1)));
//  printf("\n");

}


void HoughDetector::getEdgeData(const Vec4i &line, int &edgeLength, int &edgeStrength){
  
  edgeLength = line[1] - line[3] + 1;   // yf - yi. Only works for vertical lines!! 
  edgeStrength = 0;
  for (int y = line[3]; y <= line[1]; y++){
    edgeStrength += abs_grad_x.at<unsigned char>(y, line[0]);  
  }
}



bool HoughDetector::getEdgeData(int xmin, int xmax, int ymin, int ymax, int &size, int &strength, int &maxStrength){

  // Reproject back into the high res strip space 
  xmin = std::max(0, (xmin / XSTEP));// + 5;
  xmax = std::min((xmax / XSTEP), (iparams_.width - 1) / XSTEP);// - 5;
  ymin = std::max(0, (ymin - horizonY) / YSTEP);
  ymax = std::min((ymax - horizonY) / YSTEP, (iparams_.height - 1 - horizonY) / YSTEP);
  size = 0;
  strength = 0;
  maxStrength = 0;

  for (int x = xmin; x <= xmax; x++){
    for (int y = ymin; y <= ymax; y++){
      size++;
      int s = abs_grad_x.at<unsigned char>(y, x);
      strength += s;
      if (s >= 250)
        maxStrength++;
    }
  }

  return true;
}



void HoughDetector::findEdges(Mat &m_img, Mat &edges, int threshold_value){
  
  
  
  GaussianBlur( m_img, m_img, Size(5,5), 0, 0, BORDER_DEFAULT );
//  time = clock() - time;
//  printf("Blurring: %f sec\n", ((float)time)/CLOCKS_PER_SEC);

  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;




//  time = clock();
  Scharr( m_img, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
//  time = clock() - time;
//  printf("Scharr edge detection: %f sec\n", ((float)time)/CLOCKS_PER_SEC);
  

//  time = clock();
  convertScaleAbs( grad_x, abs_grad_x );
//  time = clock() - time;
//  printf("Convert scale abs: %f sec\n", ((float)time)/CLOCKS_PER_SEC);

//  int threshold_value = 60;         // 75 
  int max_BINARY_value = 255;

//  time = clock();
  threshold( abs_grad_x, edges, threshold_value, max_BINARY_value, 0 );
//  time = clock() - time;
//  printf("Thresholding: %f sec\n", ((float)time)/CLOCKS_PER_SEC);

}



void HoughDetector::extractMat(Mat &m_img){

//  int height = min(iparams_.height - horizonY, iparams_.height / 10);
//  height = iparams_.height - horizonY;
  for (int y = 0; y < height; y += YSTEP){
    for (int x = 0; x < iparams_.width; x += XSTEP){
      int gray;
      ColorTableMethods::xy2gray(img_, x, y + horizonY, iparams_.width, gray);
      m_img.at<unsigned char>(y/YSTEP, x/XSTEP) = gray;
//      m_img.data[m_img.step[0]*(y/YSTEP) + m_img.step[1]* (x/XSTEP)] = gray;      
    
    }
  }
}


