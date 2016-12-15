#ifndef BLOBDETECTOR_H
#define BLOBDETECTOR_H


#include <memory/TextLogger.h>
#include <vision/Constants.h>
#include <vision/ColorSegmenter.h>
#include <vision/structures/Blob.h>
#include <vision/structures/VisionParams.h>
#include <vision/Macros.h>
#include <vision/ObjectDetector.h>
#include <vision/enums/Colors.h>
  
typedef std::vector<Blob> BlobCollection;

/// @ingroup vision
class BlobDetector : public ObjectDetector {
 public:
  BlobDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter);
  void init(TextLogger* tl){textlogger = tl;};

  std::vector<BlobCollection> horizontalBlob, verticalBlob;

  void calculateHorizontalBlobData(unsigned char);
  void calculateVerticalBlobData(unsigned char);
  void verticalBlobSort(unsigned char);
  void horizontalBlobSort(unsigned char);

  void clearPointBlobReferences(Color color);
  void formWhiteLineBlobs();
  void formBlueBandBlobs();
  void formPinkBandBlobs();
  void formBlobs(Color color);
  void formBlobs(BlobCollection& blobs, Color color);
  void formOrangeBlobs();
  void formYellowBlobs();
  void resetOrangeBlobs();
  void resetYellowBlobs();
  void calculateBlobData(Color color);
  void calculateBlobData(BlobCollection& blobs, Color color);
  void calculateOrangeBlobData();
  void calculateBandBlobData(Color color);

  uint16_t mergeHorizontalBlobs(Color color, uint16_t* mergeIndex, int mergeCount);
  uint16_t mergeVerticalBlobs(Color color, uint16_t* mergeIndex, int mergeCount);
  uint16_t mergeBlobs(BlobCollection& blobs, uint16_t* mergeIndex, int mergeCount);
  uint16_t mergeBlobs(Color color, std::vector<BlobCollection>& blobs, uint16_t* mergeIndex, int mergeCount);


 private:
  void formBlobsWithVertLineSegs(unsigned char*,int);
  void formBlobsWithHorzLineSegs(unsigned char*,int);
  void setImagePointers();

  ColorSegmenter& color_segmenter_;
  VisionPoint ***verticalPoint, ***horizontalPoint;
  uint32_t **verticalPointCount, **horizontalPointCount;
  TextLogger* textlogger;
};

#endif
