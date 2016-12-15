#include <vision/ml/Util.h>
#include <yuview/YUVImage.h>
#include <common/Util.h>
#include <opencv2/highgui/highgui.hpp>

namespace util {
  cv::Mat imread(std::string path) {
    cv::Mat image;
    if(util::endswith(path, ".yuv")) {
      auto yuv = yuview::YUVImage::ReadSerializedObject(path);
      image = yuv.toMat();
    } else {
      image = ::cv::imread(path);
    }
    return image;
  }
}
