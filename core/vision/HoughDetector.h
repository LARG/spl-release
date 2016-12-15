#ifndef HOUGHDETECTOR_H
#define HOUGHDETECTOR_H

#include <vector>
#include <common/Profiling.h>
#include <vision/Constants.h>
#include <vision/structures/HorizonLine.h>
#include <vision/structures/VisionParams.h>
#include <vision/enums/Colors.h>
#include <vision/ColorTableMethods.h>
#include <vision/VisionBlocks.h>
#include <vision/structures/Blob.h>
#include <vision/ColorSegmenter.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vision/ObjectDetector.h>

class ColorSegmenter;


/*
    Some comments:

    Ideally, the mat image would be extracted only once, and flipped for the crossbar detection
    We could also just detect both horizontal and vertical lines directly on the whole image, and sort vertical versus horizontal lines for the different detections. I'm doing it like this currently because we may not want to have it classify the whole image, etc. Anyways everything will have to be rewritten... However, an advantage of splitting it like this is that the line detection, etc. requires different parameters for crossbar vs goalposts, etc.  


*/

/// @ingroup vision
class HoughDetector : public ObjectDetector {
  public:
  
    HoughDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter);
    ~HoughDetector();

  bool setHorizon(HorizonLine);
  bool extractLines(); 



  bool getEdgeData(int xmin, int xmax, int ymin, int ymax, int &size, int &strength, int &maxStrength);




  // Image coordinates of extracted image
  int getImageYMax() {return (horizonY + min(iparams_.height - horizonY, iparams_.height / 2)); }
  int getImageYMin() { return horizonY; }



  // TODO: I moved postCandidates to public because the getLines function was passing this by value, and that's causing it to be really slow. This needs to be fixed at some point.
  const std::vector<Blob>& getLines() const { return vertLines; }
  void getLines(std::vector<Blob> &lines) {lines = vertLines; }
  
  
  vector<Blob> vertLines;       // TODO: display in tool 

  bool horizonSet;


  cv::Mat m_img;
  cv::Mat edges;
  vector<cv::Vec4i> lines;

  // Subsampling step sizes for raw image
  int XSTEP;
  int YSTEP;
  int horizonY;

private:
  bool setImagePointers();
  
  ColorSegmenter& color_segmenter_;

  unsigned char* img_;
  HorizonLine horizon_;

  // Note that these are currently configured only for vertical lines
  void extractMat(cv::Mat &m_img);
  void findEdges(cv::Mat &m_img, cv::Mat &edges, int threshold_value);
  void detectLines(cv::Mat &edges, vector<cv::Vec4i> &lines);
  void filterLines(vector<cv::Vec4i> &lines, vector<Blob> &postCandidates);

  int height; 

  void getEdgeData(const cv::Vec4i &line, int &edgeLength, int &edgeStrength);


  cv::Mat grad_x, abs_grad_x;
};

struct vec4iSortByX
{
  inline bool operator() (const cv::Vec4i &v1, const cv::Vec4i &v2){
    return (v1[0] < v2[0]);
  }
};


#endif
