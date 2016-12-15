#ifndef ROI_H
#define ROI_H

#include <common/Serialization.h>
#include <schema/gen/ROI_generated.h>
#include <opencv2/core/core.hpp>
#include <common/Util.h>
#include <common/ImageParams.h>

class ColorSegmenter;

DECLARE_INTERNAL_SCHEMA(struct ROI {
  SCHEMA_METHODS(ROI);
  SCHEMA_FIELD(int xmin);
  SCHEMA_FIELD(int xmax);
  SCHEMA_FIELD(int ymin);
  SCHEMA_FIELD(int ymax);
  SCHEMA_FIELD(int xstep);
  SCHEMA_FIELD(int ystep);
  SCHEMA_FIELD(int hscale);
  SCHEMA_FIELD(int vscale);
  SCHEMA_FIELD(Camera::Type camera);
  SCHEMA_FIELD(bool valid);
  mutable SCHEMA_FIELD(cv::Mat mat);
  mutable SCHEMA_FIELD(bool extracted = false);

  ROI() { }
  ROI(int xmin, int xmax, int ymin, int ymax, Camera::Type camera) : 
    xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax), camera(camera) { }
  ROI(cv::Rect rect, Camera::Type camera) :
    xmin(rect.x), xmax(rect.x + rect.width - 1), 
    ymin(rect.y), ymax(rect.y + rect.height - 1), 
    camera(camera) { }

  ROI clone() const {
    ROI roi = *this;
    roi.mat = this->mat.clone();
    return roi;
  }

  inline int area() const { return (ymax - ymin) * (xmax - xmin); }
  inline int width() const { return xmax - xmin + 1; }
  inline int height() const { return ymax - ymin + 1; }
  inline float centerX() const { return (xmax + xmin) / 2.0f; }
  inline float centerY() const { return (ymax + ymin) / 2.0f; }

  inline std::string str() const { 
    return util::ssprintf("%s: [%04i,%04i] --> [%04i,%04i] @ (%02i,%02i)", 
      Camera::c_str(camera), xmin, ymin, xmax, ymax, xstep, ystep
    ); 
  }
  bool operator==(const ROI& other) const { 
    return xmin == other.xmin && xmax == other.xmax && ymin == other.ymin && ymax == other.ymax;
  }
  friend std::ostream& operator<<(std::ostream& os, const ROI& roi);

  void setScale(int h, int v) {
    hscale = h;
    vscale = v;
    xstep = 1 << h;
    ystep = 1 << v;
  }

  mutable ColorSegmenter* segmenter;
#ifdef TOOL
  void extract() const;
#endif
});

#endif
