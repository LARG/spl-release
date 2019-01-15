#ifndef HORIZON_LINE_H
#define HORIZON_LINE_H

#include <vision/CameraMatrix.h>
#include <stdio.h>

/// @ingroup vision
struct HorizonLine {
  HorizonLine() { exists = false; }
  bool exists;
  float gradient;
  float offset;
  static HorizonLine generate(const ImageParams& iparams, const CameraMatrix& camera, int distance);
  inline bool isAbovePoint(int imageX, int imageY) { return imageY > imageX * gradient + offset; }
  inline float getLargestYCoord(int imageWidth) {
    float y2 = offset + gradient*imageWidth;
    if (!exists || offset < 0 && y2 <0) {
        return 0.0;
    }
    return std::max(offset, y2);
  }
  inline void writeHorizon(char* logname, int imageWidth, int counter, bool isBottom) {
    char buff[100];
    if (isBottom) {
      snprintf(buff, sizeof(buff), "/home/jkelle/nao/trunk/BlobImages/%s/bottom/horizon_%03i.txt", logname, counter);      
    } else {
      snprintf(buff, sizeof(buff), "/home/jkelle/nao/trunk/BlobImages/%s/top/horizon_%03i.txt", logname, counter);
    }
    std::string filepath = buff;
    ofstream file;
    file.open(filepath);

    if (exists) {
      file << "offset\n" <<  offset;
      file << "\ngradient\n" <<  gradient;
    } else {
      file << "offset\n" <<  0;
      file << "\ngradient\n" <<  0;
    }

    file.close();
  }

  inline int getYGivenX(int x){
    return gradient * x + offset; 
  }


};

#endif
