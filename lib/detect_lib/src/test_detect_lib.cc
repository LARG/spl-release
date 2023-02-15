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

#include "detect_lib.h"
#include <yaml-cpp/yaml.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "read_detect_lib_config.h"

using namespace cv;
using namespace std;

#define NUM_THREADS 4
#define NUM_TIMING_LOOPS 2 
#define BYTES_TO_IGNORE 36
typedef unsigned char BYTE;


double get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }


#define TFLITE_MINIMAL_CHECK(x)                              \
  if (!(x))                                                  \
  {                                                          \
    fprintf(stderr, "Error at %s:%d\n", __FILE__, __LINE__); \
    exit(1);                                                 \
  }


template <class T> std::vector<T> getSlice(std::vector<T> vec, unsigned int start, unsigned int end, unsigned int step)
{
  std::vector<T> slice; 
  for(unsigned int i=start; i<end; i+=step)
    slice.push_back(vec[i]);
  return slice;
}

template <class T> void setSlice(std::vector<T> &vec_target, std::vector<T> &vec_source, unsigned int start, unsigned int end, unsigned int step)
{
  unsigned int count = 0;
  for(unsigned int i=start; i<end; i+=step)
  {
    vec_target[i] = vec_source[count];
    count++;
  }
}

std::vector<BYTE> readFile(char* filename)
{
    // open the file:
    std::streampos fileSize;
    std::ifstream file(filename, std::ios::binary);

    // get its size:
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // read the data:
    std::vector<BYTE> fileData(fileSize);
    file.read((char*) &fileData[0], fileSize);
    return fileData;
}


// Function to convert a byte stream containing the raw log data to
// UYVY format which OpenCV can then convert. The process is to
// drop the first 36 or so bytes from the stream, then interleave
// the rest of the stream into separate channels.
cv::Mat Bytes2UYVY(std::vector<BYTE> bytes, unsigned int width, unsigned int height, unsigned int bytes_to_drop)
{
  auto Y1 = getSlice<BYTE>(bytes, bytes_to_drop + 0, bytes.size(), 4);
  auto U = getSlice<BYTE>(bytes, bytes_to_drop + 1, bytes.size(), 4);
  auto Y2 = getSlice<BYTE>(bytes, bytes_to_drop + 2, bytes.size(), 4);
  auto V = getSlice<BYTE>(bytes, bytes_to_drop + 3, bytes.size(), 4);

  auto UV = std::vector<BYTE>((width * height), 0);
  auto YY = std::vector<BYTE>((width * height), 0);

  // UV[0::2] = np.fromstring(U,  dtype=np.uint8)
  setSlice<BYTE>(UV, U, 0, UV.size(), 2);
  // UV[1::2] = np.fromstring(V,  dtype=np.uint8)
  setSlice<BYTE>(UV, V, 1, UV.size(), 2);
  // YY[0::2] = np.fromstring(Y1, dtype=np.uint8)
  setSlice<BYTE>(YY, Y1, 0, YY.size(), 2);
  // YY[1::2] = np.fromstring(Y2, dtype=np.uint8)
  setSlice<BYTE>(YY, Y2, 1, YY.size(), 2);

  // UV = UV.reshape((self.frame_height, self.frame_width))
  auto mat_UV = cv::Mat(height, width, CV_8UC1, UV.data());
  // YY = YY.reshape((self.frame_height, self.frame_width))
  auto mat_YY = cv::Mat(height, width, CV_8UC1, YY.data());

  // frame_uyvy = cv2.merge([UV, YY])
  cv::Mat frame_uyvy;
  vector<cv::Mat> channels;
  channels.push_back(mat_UV);
  channels.push_back(mat_YY);
  cv::merge(channels, frame_uyvy);

  return frame_uyvy;
}

// Call OpenCV to convert a UYVY frame to RGB
cv::Mat UYVY2RGB(cv::Mat frame_uyvy)
{
  //frame_bgr = cv2.cvtColor(frame_uyvy, cv2.COLOR_YUV2BGR_UYVY)
  cv::Mat frame_rgb = cv::Mat(frame_uyvy.size().height, frame_uyvy.size().width, CV_8UC3);
  cv::cvtColor(frame_uyvy, frame_rgb, cv::COLOR_YUV2RGB_UYVY);
  return frame_rgb;
}

cv::Mat ReadYUV2RGB(char * filename, unsigned int width, unsigned int height, unsigned int bytes_to_drop)
{
  auto bytes = readFile(filename);
  return UYVY2RGB(Bytes2UYVY(bytes, width, height, bytes_to_drop));
}


void DrawRectangles(
  cv::Mat & img,
  const vector<vector<float>> & vecVecFloat,
  float scale = 1.0
) {
  for (const auto & vec: vecVecFloat) {
    cv::Rect rect = cv::Rect(cv::Point(scale*vec[0], scale*vec[1]),
    cv::Point(scale*vec[2], scale*vec[3]));
    cv::rectangle(img, rect,  cv::Scalar(0, 0, 255), 1);
  }
}


int main(int argc, char *argv[])
{
  unsigned int width = 224;
  unsigned int height = 120;
  unsigned int num_output_boxes = 420;
  // unsigned int num_output_boxes = 9 * 9 * 2;
  int num_classes = 2;
  float min_confidence = 0.5;

  if (argc < 6)
  {
    // Example usage: ./detect_yuv ../tflite_models/micro.tflite ../samples/top_0012.yuv 1280 960 36
    fprintf(stderr, "detect <tflite model> <image.yuv> <width> <height> <bytes_to_drop> optional[<input_w> <input_h> <num_output_boxes>]\n");
    return 1;
  }
  char *detect_lib_config_file = argv[1];
  char *img_file =   argv[2];

  unsigned int input_width = atoi(argv[3]);
  unsigned int input_height = atoi(argv[4]);
  unsigned int bytes_to_drop = atoi(argv[5]);


  DetectLibConfig *cfg = yaml_to_config(detect_lib_config_file);

  std::string nao_home_path(getenv("NAO_HOME"));
  cfg->model_path = nao_home_path + cfg->model_path;
  
  // if (argc > 6)
  // {
  //   width = atoi(argv[6]);
  //   height = atoi(argv[7]);
  //   num_output_boxes = atoi(argv[8]);
  // }


  width = (cfg->input_width)/(cfg->subsample_major + cfg->subsample_minor);
  height = (cfg->input_height)/(cfg->subsample_major + cfg->subsample_minor);
  num_output_boxes = cfg->num_output_boxes;
  num_classes = cfg->num_classes;

  cout << "Yaml file: " << detect_lib_config_file << endl;
  cout << "Image file: " << img_file << endl;
  cout << "Input size: " << width << "x" << height << "px" << endl;

    // Read and resize the image using opencv
  // cv::Mat cvimg = cv::imread(img_file);
  cv::Mat cvimg = ReadYUV2RGB(img_file, input_width, input_height, bytes_to_drop);
  cout << "Image read... ok." << endl;
  cv::Mat cvimg_bgr = cvimg.clone();
  cv::cvtColor(cvimg_bgr, cvimg_bgr, CV_RGB2BGR);
  cv::imwrite("../samples/converted.png", cvimg_bgr);

  // DetectLibConfig *cfg = new DetectLibConfig;
  // cfg->model_path = std::string(model_file);
  // cfg->input_width = input_width;
  // cfg->input_height = input_height;
  // cfg->num_output_boxes = num_output_boxes;
  // cfg->confidence_threshold = min_confidence;
  // cfg->num_classes = 1;
  // cfg->threads = 4;
  // cfg->subsample_major = 3;
  // cfg->subsample_minor = 2;
  // cfg->exact_subsample= false;

  auto img_yuyv = readFile(img_file);

  if (cvimg.data == NULL)
  {
    printf("=== IMAGE READ ERROR ===\n");
    return 0;
  }
// #if (CV_VERSION_MAJOR >= 4)
//   cv::cvtColor(cvimg, cvimg, COLOR_BGR2RGB);
// #else
//   cv::cvtColor(cvimg, cvimg, CV_BGR2RGB);
// #endif
  // cv::resize(cvimg, cvimg, Size(width, height));
  // cout << "Image resize... ok." << endl;

  unique_ptr<TFLiteDetector> D;
  D = make_unique<TFLiteDetector>(cfg);
    struct timeval start_time, stop_time;
  gettimeofday(&start_time, nullptr);

  vector<float> best_box;
  vector<vector<float>> rects;
  vector<vector<float>> rects_center;

  const vector<string> cnames = {"ball", "cross", "robot"};

  for (int i = 0; i < NUM_TIMING_LOOPS; i++) {
  // best_box = D->DetectRGB(cvimg);
  // best_box = D->DetectDirect(img_yuyv.data()+bytes_to_drop);
    rects = D->DetectMulticlass(img_yuyv.data()+bytes_to_drop, true);
  }

  gettimeofday(&stop_time, nullptr);
  std::cout << "invoked\n";
  std::cout << "average time: "
            << (get_us(stop_time) - get_us(start_time)) /
                   (NUM_TIMING_LOOPS * 1000)
            << " ms\n";

  cv::Mat img_after = cvimg.clone();
  auto scale = 0.5*(cfg->subsample_major + cfg->subsample_minor);
  for(auto& box : rects)
  {
    char buffer [50];
    sprintf(buffer, "%s %.2f %.2f %.2f %.2f %.2f", cnames[((int)(box[5]))].c_str(), box[4], box[0], box[1], box[2], box[3]);
    string msg = string(buffer);
    std::cout << msg << endl;
    auto xmin = box[0] - 0.5f * box[2];
    auto ymin = box[1];
    cv::putText(
      img_after, msg, Point(scale*xmin, scale*ymin),
      FONT_HERSHEY_DUPLEX, 0.4, Scalar(0, 0, 255),
      1, false
    );
    rects_center.push_back({
      box[0] - 0.5f * box[2],
      box[1] - 0.5f * box[3],
      box[0] + 0.5f * box[2],
      box[1] + 0.5f * box[3],
    });
  }
  DrawRectangles(img_after, rects_center, scale);
  cv::cvtColor(img_after, img_after, CV_RGB2BGR);
  cv::imwrite("../samples/result.png", img_after);

  // if (best_box.size() == 0) {
  //   cout << "No ball found" << endl;
  // } else{
  //   // draw rectangle in image
  //   cv::Mat img_after = cvimg.clone();
  //   vector<vector<float>> rectangles = {
  //     {
  //       best_box[0] - 0.5f * best_box[2],
  //       best_box[1] - 0.5f * best_box[3],
  //       best_box[0] + 0.5f * best_box[2],
  //       best_box[1] + 0.5f * best_box[3],
  //     }
  //   };
  //   cout << "Found a ball centered at " <<
  //     "(" << best_box[0] << ", " << best_box[1] << ")" <<
  //     " width width and height " <<
  //     best_box[2] << "x" << best_box[3] <<
  //     "\n with confidence " << best_box[4] <<endl;
  //   DrawRectangles(img_after, rectangles);
  //   cv::imwrite("../samples/result.png", img_after);
  // }

  return 0;
}
