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

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"

#include "tensorflow/lite/delegates/xnnpack/xnnpack_delegate.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/optional_debug_tools.h"
#include "tensorflow/lite/profiling/profiler.h"
#include "tensorflow/lite/string_util.h"
#include "tensorflow/lite/tools/command_line_flags.h"
#include "tensorflow/lite/tools/delegates/delegate_provider.h"
#include "tensorflow/lite/tools/evaluation/utils.h"

#ifndef TOOL

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

#endif


using namespace std;

#define NUM_THREADS 4


#define TFLITE_MINIMAL_CHECK(x)                              \
  if (!(x))                                                  \
  {                                                          \
    fprintf(stderr, "Error at %s:%d\n", __FILE__, __LINE__); \
    exit(1);                                                 \
  }


/*
  ================================
  Non-max suppression code based on: https://github.com/martinkersner/non-maximum-suppression-cpp/blob/master/nms.cpp
  Starts here.
  ================================
*/

enum PointInRectangle {XMIN, YMIN, XMAX, YMAX};


vector<float> GetPointFromXYWH(const vector<vector<float>> & xywh,
                              const PointInRectangle & pos)
{
  vector<float> points;
  
  for (const auto & coords: xywh) {
    const float x = coords[0];
    const float y = coords[1];
    const float w = coords[2];
    const float h = coords[3];
    switch (pos)
    {
    case 0:
      points.push_back(x - 0.5 * w);
      break;
    case 1:
      points.push_back(y - 0.5 * h);
      break;
    case 2:
      points.push_back(x + 0.5 * w);
      break;
    case 3:
      points.push_back(y + 0.5 * h);
      break;
    }
  }
  return points;
}


vector<float> ComputeArea(const vector<float> & x1,
                          const vector<float> & y1,
                          const vector<float> & x2,
                          const vector<float> & y2)
{
  vector<float> area;
  auto len = x1.size();
  
  for (decltype(len) idx = 0; idx < len; ++idx) {
    auto tmpArea = (x2[idx] - x1[idx] + 1) * (y2[idx] - y1[idx] + 1);
    area.push_back(tmpArea);
  }
  
  return area;
}

template <typename T>
vector<int> argsort(const vector<T> & v)
{
  // initialize original index locations
  vector<int> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);
  
  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](int i1, int i2) {return v[i1] < v[i2];});
  
  return idx;
}

vector<float> Maximum(const float & num,
                      const vector<float> & vec)
{
  auto maxVec = vec;
  auto len = vec.size();
  
  for (decltype(len) idx = 0; idx < len; ++idx)
    if (vec[idx] < num)
      maxVec[idx] = num;
  
  return maxVec;
}

vector<float> Minimum(const float & num,
                      const vector<float> & vec)
{
  auto minVec = vec;
  auto len = vec.size();
  
  for (decltype(len) idx = 0; idx < len; ++idx)
    if (vec[idx] > num)
      minVec[idx] = num;
  
  return minVec;
}

vector<float> CopyByIndexes(const vector<float> & vec,
                            const vector<int> & idxs)
{
  vector<float> resultVec;
  
  for (const auto & idx : idxs)
    resultVec.push_back(vec[idx]);
  
  return resultVec;
}

vector<int> RemoveLast(const vector<int> & vec)
{
  auto resultVec = vec;
  resultVec.erase(resultVec.end()-1);
  return resultVec;
}

vector<float> Subtract(const vector<float> & vec1,
                       const vector<float> & vec2)
{
  vector<float> result;
  auto len = vec1.size();
  
  for (decltype(len) idx = 0; idx < len; ++idx)
    result.push_back(vec1[idx] - vec2[idx] + 1);
  
  return result;
}

vector<float> Multiply(const vector<float> & vec1,
		                   const vector<float> & vec2)
{
  vector<float> resultVec;
  auto len = vec1.size();
  
  for (decltype(len) idx = 0; idx < len; ++idx)
    resultVec.push_back(vec1[idx] * vec2[idx]);
  
  return resultVec;
}

vector<float> Divide(const vector<float> & vec1,
		                 const vector<float> & vec2)
{
  vector<float> resultVec;
  auto len = vec1.size();
  
  for (decltype(len) idx = 0; idx < len; ++idx)
    resultVec.push_back(vec1[idx] / vec2[idx]);
  
  return resultVec;
}

vector<int> WhereLarger(const vector<float> & vec,
                        const float & threshold)
{
  vector<int> resultVec;
  auto len = vec.size();
  
  for (decltype(len) idx = 0; idx < len; ++idx)
    if (vec[idx] > threshold)
      resultVec.push_back(idx);
  
  return resultVec;
}

vector<int> RemoveByIndexes(const vector<int> & vec,
                            const vector<int> & idxs)
{
  auto resultVec = vec;
  auto offset = 0;
  
  for (const auto & idx : idxs) {
    resultVec.erase(resultVec.begin() + idx + offset);
    offset -= 1;
  }
  
  return resultVec;
}


template <typename T>
vector<T> FilterVector(const vector<T> & vec,
    const vector<int> & idxs)
{
  vector<T> resultVec;
  
  for (const auto & idx: idxs)
    resultVec.push_back(vec[idx]);
  
  return resultVec;
}


vector<int> nms(
  const vector<vector<float>> & boxes,
  const float & threshold)
{
  if (boxes.empty())
  	return vector<int>();
  // grab the coordinates of the bounding boxes
  auto x1 = GetPointFromXYWH(boxes, XMIN);
  auto y1 = GetPointFromXYWH(boxes, YMIN);
  auto x2 = GetPointFromXYWH(boxes, XMAX);
  auto y2 = GetPointFromXYWH(boxes, YMAX);
  
  // compute the area of the bounding boxes and sort the bounding
  // boxes by the bottom-right y-coordinate of the bounding box
  auto area = ComputeArea(x1, y1, x2, y2);
  auto idxs = argsort(y2);
  
  int last;
  int i;
  vector<int> pick;
  
  // keep looping while some indexes still remain in the indexes list
  while (idxs.size() > 0) {
    // grab the last index in the indexes list and add the
    // index value to the list of picked indexes
    last = idxs.size() - 1;	
    i    = idxs[last];
    pick.push_back(i);
    
    // find the largest (x, y) coordinates for the start of
    // the bounding box and the smallest (x, y) coordinates
    // for the end of the bounding box
    auto idxsWoLast = RemoveLast(idxs);

    auto xx1 = Maximum(x1[i], CopyByIndexes(x1, idxsWoLast));
    auto yy1 = Maximum(y1[i], CopyByIndexes(y1, idxsWoLast));
    auto xx2 = Minimum(x2[i], CopyByIndexes(x2, idxsWoLast));
    auto yy2 = Minimum(y2[i], CopyByIndexes(y2, idxsWoLast));

		// compute the width and height of the bounding box
    auto w = Maximum(0, Subtract(xx2, xx1));
    auto h = Maximum(0, Subtract(yy2, yy1));
		
		// compute the ratio of overlap
    auto overlap = Divide(Multiply(w, h), CopyByIndexes(area, idxsWoLast));

    // delete all indexes from the index list that have
    auto deleteIdxs = WhereLarger(overlap, threshold);
    deleteIdxs.push_back(last);
    idxs = RemoveByIndexes(idxs, deleteIdxs);
  }

  return pick;
}

/*
  ==================================
  Non-max suppression code ends here.
  ==================================
*/

// YAML::Node parse_yaml(std::istream& input) {
//   YAML::Node doc;
//   try {
//     doc = YAML::Load(input);
//   } catch (const YAML::Exception& e) {
//     std::cerr << e.what() << "\n";
//   }
//   return doc;
// }

struct InterpreterStorage{
    std::unique_ptr<tflite::Interpreter> interpreter;
    tflite::evaluation::TfLiteDelegatePtr delegate = tflite::evaluation::TfLiteDelegatePtr(nullptr, [](TfLiteDelegate*) {});
    std::unique_ptr<tflite::FlatBufferModel> model;
    tflite::ops::builtin::BuiltinOpResolver resolver;
    DetectLibConfig * config;
    // tflite::InterpreterBuilder builder;
};

TFLiteDetector::TFLiteDetector(){}

TFLiteDetector::TFLiteDetector(DetectLibConfig * cfg)
{
  S = std::make_unique<InterpreterStorage>();
  S->config = cfg;
  auto model_file_str = S->config->model_path;
  const char * model_file = model_file_str.c_str();
  cout << "TFLite model file:" << model_file << endl;
  // Load model
  // std::unique_ptr<tflite::FlatBufferModel> model =
  //     tflite::FlatBufferModel::BuildFromFile(model_file);
  S->model = tflite::FlatBufferModel::BuildFromFile(model_file);
  TFLITE_MINIMAL_CHECK((S->model) != nullptr);

  // Build the interpreter with the InterpreterBuilder.
  // Note: all Interpreters should be built with the InterpreterBuilder,
  // which allocates memory for the Intrepter and does various set up
  // tasks so that the Interpreter can read the provided model.
  // tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder builder(*(S->model), S->resolver);
  builder(&(S->interpreter), 1);
  TFLITE_MINIMAL_CHECK((S->interpreter) != nullptr);


  // Create the delegate object using the helpers provided in TFLite.
  // Note that this comes from a different header that needs to be included
  // Separately.
  // Here we are creating a "standard" XNNPack CPU delegate and we can
  // specify the number of threads (hardcoded here).
  // auto delegate = tflite::evaluation::CreateXNNPACKDelegate(NUM_THREADS);
  // S->delegate = (tflite::evaluation::CreateXNNPACKDelegate(S->config->threads));
  // if (!(S->delegate)) {
  //     std::cerr << "XNNPACK acceleration is unsupported on this platform.";
  // } else {
  //     std::cout << "XNNPACK delegate created\n";
  // }
  { using namespace tflite;
  { using namespace evaluation;
  
  if (S->config->enable_gpu)
  {
    std::cout << "Initializing GPU Delegate\n";
    TfLiteGpuDelegateOptionsV2 options = TfLiteGpuDelegateOptionsV2Default();
    options.inference_priority1 = TFLITE_GPU_INFERENCE_PRIORITY_MIN_LATENCY;
    options.inference_priority2 = TFLITE_GPU_INFERENCE_PRIORITY_AUTO;
    options.inference_priority3 = TFLITE_GPU_INFERENCE_PRIORITY_AUTO;
    options.inference_preference =
        TFLITE_GPU_INFERENCE_PREFERENCE_SUSTAINED_SPEED;
    S->delegate = (CreateGPUDelegate(&options));
    if (!(S->delegate)) {
        std::cerr << "GPU acceleration is unsupported on this platform.";
    } else {
        std::cout << "GPU delegate created\n";
    }
  }
  else
  {
    std::cout << "Initializing XNNPack Delegate\n";
    S->delegate = (tflite::evaluation::CreateXNNPACKDelegate(S->config->threads));
    if (!(S->delegate)) {
        std::cerr << "XNNPACK acceleration is unsupported on this platform.";
    } else {
        std::cout << "XNNPACK delegate created\n";
    }
  }

  }
  }



  // This step is necessary to make it use the delegate. Not clear
  // if it can be performed later or it has to be done now.
  // After this, everything is as usual.
  if (((S->interpreter))->ModifyGraphWithDelegate(((S->delegate)).get()) !=
      kTfLiteOk) {
    std::cerr << "Failed to apply " << "XNNPack" << " delegate.\n";
    exit(-1);
  } else {
    std::cout << "Successfully Applied " << "XNNPack" << " delegate.\n";
  }

  // Allocate tensor buffers.
  TFLITE_MINIMAL_CHECK(((S->interpreter))->AllocateTensors() == kTfLiteOk);
  TFLITE_MINIMAL_CHECK(((S->interpreter))->Invoke() == kTfLiteOk);
  cout << "Init inference test... ok." << endl;
}

TFLiteDetector::~TFLiteDetector()
{
}

vector<vector<float> > TFLiteDetector::DetectMulticlass(const unsigned char *imgraw,
                                                        bool debug_output)
{
  auto config = *(S->config);
  unsigned int num_output_boxes= config.num_output_boxes;
  float min_confidence = config.confidence_threshold;
  unsigned int num_classes = config.num_classes;
  unsigned int subsample_major = config.subsample_major;
  unsigned int subsample_minor = config.subsample_minor;
  unsigned int input_channels = config.input_channels;
  unsigned int skip_input_channel = config.skip_input_channel;
  bool ess = config.exact_subsample;

  int i = 0;
  bool sr=true, sc=true;
  int pr = 0;
  
  // Define these things so that we don't have to compute them during the loop
  
  int shift_r_major = (2*config.input_width)*(subsample_major-1);
  int shift_r_minor = (2*config.input_width)*(subsample_minor-1);

  int shift_c_major = 4*(subsample_major-1);
  int shift_c_minor = 4*(subsample_minor-1);

  int inc_c_major = 2*subsample_major;
  int inc_c_minor = 2*subsample_minor;


  int inc_r = (sr?subsample_major:subsample_minor);
  float * input_tensor = ((S->interpreter))->typed_input_tensor<float>(0);
  // Fill tf-lite input tensor
  for(int r = 0; r < config.input_height;) {
    // IMPORTANT: Here CNN's input width is twice the YUYV image width!!
    // UNLESS: Exact subsample
    // int acc = 0;
    for(int c = 0; c < config.input_width;) {
      // Fill channels YUYV
      // input_tensor[i++]
      // //  = ((uint8_t) (*(imgraw++))); // Y0
      //  = ((*(imgraw++))); // Y0
      // input_tensor[i++]
      // //  = ((uint8_t) (*(imgraw++))); // U
      //  = ((*(imgraw++))); // U
      // input_tensor[i++]
      // //  = ((uint8_t) (*(imgraw++))); // Y1
      //  = ((*(imgraw++))); // Y1
      // input_tensor[i++]
      // //  = ((uint8_t) (*(imgraw++))); // V
      //  = ((*(imgraw++))); // V
      // i += 4;
      for(int b=0; b<input_channels; ++b, ++imgraw)
        if(b != skip_input_channel)
          *(input_tensor++) = ((*(imgraw)));
      if(sc)
      {
        c += inc_c_major;
        imgraw += shift_c_major;
      }
      else
      {
        c += inc_c_minor;
        imgraw += shift_c_minor;
      }
      sc = (!sc);
      // int impl_inc = (sc?subsample_major:subsample_minor);
      // impl_inc += impl_inc; // mult by 2
      // c += impl_inc;
      // impl_inc += (impl_inc - 4); // 4(x -1)
      // imgraw += impl_inc;
      // sc = (!sc);
      // int ss_steps = (sc?subsample_major:subsample_minor);
      // int impl_inc = (pr+ ss_steps);
      // pr = impl_inc%2;
      // c += (1+!(ess))*ss_steps;
      // imgraw += 4*((ess?(impl_inc/2):ss_steps)-1);
      // sc = (!sc);
      // acc += ((ess?(impl_inc/2):ss_steps));
      // cout << r << " " << c << " impl inc " << impl_inc << endl;
    }
    r += inc_r;
    sr = (!sr);
    if(sr)
    {
      inc_r = subsample_major;
      imgraw += shift_r_major;
    }
    else
    {
      inc_r = subsample_minor;
      imgraw += shift_r_minor;
    }
  }

  if(debug_output)
  {
    cout << "Copied image to tensor... ok." << endl;
    cout << std::flush;
  }

  // Run inference
  TFLITE_MINIMAL_CHECK(((S->interpreter))->Invoke() == kTfLiteOk);
  if(debug_output)
  {
    cout << "Finished inference... ok." << endl;
    cout << std::flush;
  }

  // Read output tensor
  float wr=1.0, hr=1.0;
  if(config.output_scale)
  {
    wr = config.input_width / (0.5*(config.subsample_major + config.subsample_minor));
    hr = config.input_height / (0.5*(config.subsample_major + config.subsample_minor));
  }

  // Process outputs for each class
  vector<vector<float>> boxes;
  vector<float> confidence;
  vector<int> cls;

  float * output_tensor = (S->interpreter)->typed_output_tensor<float>(0);

  for (auto i = 0; i < num_output_boxes; ++i) {
    int ix = (5 + num_classes) * i;
    float conf = output_tensor[ix + 4];
    if (conf < min_confidence)
      continue;
    // must find the max class
    int best_class = -1;
    float best_prob = -100.0;
    for (auto c=0; c < num_classes; c++) {
      float class_prob = output_tensor[ix + 5 + c];
      if (class_prob > best_prob) {
        best_class = c;
        best_prob = class_prob;
      }
    }
    confidence.push_back(conf);
    cls.push_back(best_class);

    vector<float> box = {
      wr * output_tensor[ix++],
      hr * output_tensor[ix++],
      wr * output_tensor[ix++],
      hr * output_tensor[ix++]
    };
    boxes.push_back(box);
  }
  int num_found_boxes = boxes.size();
  // printf("Extracted output tensor data\n");
  // printf("Found %d boxes above min confidence.\n", num_found_boxes);

  // // class agnostic non-max spression
  // const float threshold = 0.5;
  vector<int> pick = nms(boxes, min_confidence);
  vector<vector<float>> fboxes = FilterVector(boxes, pick);
  vector<float> fconfidence = FilterVector(confidence, pick);
  vector<int> fcls = FilterVector(cls, pick);
  
  int num_found_boxes_post_nms = fboxes.size();
  if(debug_output)
    printf("Found %d boxes above min confidence after nms.\n", num_found_boxes_post_nms);
  vector<vector<float>> rects;
  for (auto i = 0; i < fboxes.size(); i++) {
    auto & box = fboxes[i];
    if(debug_output)
      printf("Class %d, conf %.3f at %.2f, %.2f \n", fcls[i], fconfidence[i], box[0], box[1]);
    vector<float> rect = {
      // box[0] - 0.5f * box[2],
      // box[1] - 0.5f * box[3],
      // box[0] + 0.5f * box[2],
      // box[1] + 0.5f * box[3],
      box[0],
      box[1],
      box[2],
      box[3],
      fconfidence[i],
      fcls[i],
    };
    rects.push_back(rect);
  }
  return rects;
}

vector<float> TFLiteDetector::DetectDirect(const unsigned char *imgraw)
{
  auto config = *(S->config);
  unsigned int num_output_boxes= config.num_output_boxes;
  float min_confidence = config.confidence_threshold;
  unsigned int num_classes = 1;
  unsigned int subsample_major = config.subsample_major;
  unsigned int subsample_minor = config.subsample_minor;

  vector<vector<float>> boxes;
  vector<float> confidence;

  int i = 0;
  bool sr=true, sc=true;
  // Fill tf-lite input tensor
  for(int r = 0; r < config.input_height;) {
    // IMPORTANT: Here input width is half the image width!!
    for(int c = 0; c < config.input_width;) {
      // Fill channels YUYV
      ((S->interpreter))->typed_input_tensor<float>(0)[i]
      //  = ((uint8_t) (*(imgraw++))); // Y0
       = ((*(imgraw++))); // Y0
      ((S->interpreter))->typed_input_tensor<float>(0)[i + 1]
      //  = ((uint8_t) (*(imgraw++))); // U
       = ((*(imgraw++))); // U
      ((S->interpreter))->typed_input_tensor<float>(0)[i + 2]
      //  = ((uint8_t) (*(imgraw++))); // Y1
       = ((*(imgraw++))); // Y1
      ((S->interpreter))->typed_input_tensor<float>(0)[i + 3]
      //  = ((uint8_t) (*(imgraw++))); // V
       = ((*(imgraw++))); // V
      i += 4;
      c += 2*(sc?subsample_major:subsample_minor);
      imgraw += 4*((sc?subsample_major:subsample_minor)-1);
      sc = (!sc);
      // cout << r << " " << c << endl;
    }
    r += (sr?subsample_major:subsample_minor);
    sr = (!sr);
    imgraw += 2*config.input_width*((sr?subsample_major:subsample_minor)-1);
  }

  // cout << "Copied image to tensor... ok." << endl;

  // Run inference
  TFLITE_MINIMAL_CHECK(((S->interpreter))->Invoke() == kTfLiteOk);
  // cout << "Finished inference... ok." << endl;

  // Read output tensor
  for (auto i = 0; i < num_output_boxes; ++i) {
    float conf = ((S->interpreter))->typed_output_tensor<float>(0)[(5 + num_classes) * i + 4];
    if (conf < min_confidence)
      continue;
    confidence.push_back(conf);
    vector<float> box = {
      ((S->interpreter))->typed_output_tensor<float>(0)[(5 + num_classes) * i + 0],
      ((S->interpreter))->typed_output_tensor<float>(0)[(5 + num_classes) * i + 1],
      ((S->interpreter))->typed_output_tensor<float>(0)[(5 + num_classes) * i + 2],
      ((S->interpreter))->typed_output_tensor<float>(0)[(5 + num_classes) * i + 3],
      conf
    };
    boxes.push_back(box);
  }
  // cout << "Extracted output tensor data to vector of vectors... ok." << endl;

  vector<float> best_box;
  if (boxes.size() == 0) {
    // cout << "No ball found" << endl;
  } else{
    // scan for max
    int best_id = 0;
    float best_conf = confidence[0];
    for (int i = 0; i < boxes.size(); i++) {
      if (confidence[i] > best_conf) {
        best_id = i;
        best_conf = confidence[i];
      }
    }
    best_box = boxes[best_id];
  }
  return best_box;
}

#ifndef TOOL

vector<float> TFLiteDetector::DetectRGB(cv::Mat cvimg
)
{
  auto config = *(S->config);
  unsigned int num_output_boxes= config.num_output_boxes;
  float min_confidence = config.confidence_threshold;

  vector<vector<float>> boxes;
  vector<float> confidence;

  cv::Mat cvimg_resized;
  cv::resize(cvimg, cvimg_resized, cv::Size(config.input_width, config.input_height));
  unsigned int num_classes = 1;
  cv::Size s = cvimg_resized.size();

  // Fill tf-lite input tensor
  for (int i=0; i < s.height; ++i) {
    for (int j=0; j < s.width; ++j) {
      auto &rgb = cvimg_resized.at<cv::Vec3b>(i, j);
      ((S->interpreter))->typed_input_tensor<float>(0)[3 * (i * s.width + j) + 0]
       = float(rgb[0]) / 255.0f;
      ((S->interpreter))->typed_input_tensor<float>(0)[3 * (i * s.width + j) + 1]
       = float(rgb[1]) / 255.0f;
      ((S->interpreter))->typed_input_tensor<float>(0)[3 * (i * s.width + j) + 2]
       = float(rgb[2]) / 255.0f;
    }
  }
  // cout << "Copied image to tensor... ok." << endl;

  // Run inference
  TFLITE_MINIMAL_CHECK(((S->interpreter))->Invoke() == kTfLiteOk);
  // printf("\n\n=== Post-invoke Interpreter State ===\n");
  // tflite::PrintInterpreterState(interpreter.get());
  // cout << "Finished inference... ok." << endl;

  // Read output tensor
  for (auto i = 0; i < num_output_boxes; ++i) {
    float conf = ((S->interpreter))->typed_output_tensor<float>(0)[(5 + num_classes) * i + 4];
    if (conf < min_confidence)
      continue;
    confidence.push_back(conf);
    vector<float> box = {
      ((S->interpreter))->typed_output_tensor<float>(0)[(5 + num_classes) * i + 0],
      ((S->interpreter))->typed_output_tensor<float>(0)[(5 + num_classes) * i + 1],
      ((S->interpreter))->typed_output_tensor<float>(0)[(5 + num_classes) * i + 2],
      ((S->interpreter))->typed_output_tensor<float>(0)[(5 + num_classes) * i + 3],
      conf
    };
    boxes.push_back(box);
    // for (auto it = box.begin(); it != box.end(); it++)
    //   cout << *it << " ";
    // cout << endl;
  }
  // cout << "Extracted output tensor data to vector of vectors... ok." << endl;

  vector<float> best_box;
  if (boxes.size() == 0) {
    // cout << "No ball found" << endl;
  } else{
    // scan for max
    int best_id = 0;
    float best_conf = confidence[0];
    for (int i = 0; i < boxes.size(); i++) {
      if (confidence[i] > best_conf) {
        best_id = i;
        best_conf = confidence[i];
      }
    }
    best_box = boxes[best_id];
    // cout << "Found a ball centered at " <<
    //   "(" << best_box[0] << ", " << best_box[1] << ")" <<
    //   " width width and height " <<
    //   best_box[2] << "x" << best_box[3] <<  
    //   " and confidence " << best_conf << endl;
  }
  return best_box;
}

#endif
