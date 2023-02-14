#include <fcntl.h>      // NOLINT(build/include_order)
#include <getopt.h>     // NOLINT(build/include_order)
#include <sys/time.h>   // NOLINT(build/include_order)
#include <sys/types.h>  // NOLINT(build/include_order)
#include <sys/uio.h>    // NOLINT(build/include_order)
#include <unistd.h>     // NOLINT(build/include_order)

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#ifndef TOOL

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#endif


class InterpreterStorage;

struct DetectLibConfig
{
  std::string model_path;
  unsigned int input_width, input_height;
  unsigned int subsample_major, subsample_minor;
  bool exact_subsample=false;
  unsigned int num_output_boxes;
  unsigned int num_classes;
  unsigned int threads;
  unsigned int input_channels;
  unsigned int skip_input_channel;
  float confidence_threshold;
  bool enable_gpu=false;
  bool output_scale=false;
};

class TFLiteDetector
{
private:
  std::unique_ptr<InterpreterStorage> S;
public:
  TFLiteDetector();
  TFLiteDetector(DetectLibConfig*);
  ~TFLiteDetector();
  #ifndef TOOL
  std::vector<float> DetectRGB(cv::Mat);
  #endif
  std::vector<float> DetectDirect(const unsigned char *);
  std::vector<std::vector<float> > DetectMulticlass(
    const unsigned char *,
    bool debug_output=false);
};


