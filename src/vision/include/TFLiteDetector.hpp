#pragma once
#include <ObjectDetector.hpp>

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


class TFLiteDetector : public ObjectDetector
{
    std::unique_ptr<tflite::Interpreter> interpreter;
    tflite::evaluation::TfLiteDelegatePtr delegate = tflite::evaluation::TfLiteDelegatePtr(nullptr, [](TfLiteDelegate*) {});
    std::unique_ptr<tflite::FlatBufferModel> model;
    tflite::ops::builtin::BuiltinOpResolver resolver;

    std::string model_path;
    unsigned int input_width, input_height;
    unsigned int model_input_width, model_input_height;
    unsigned int subsample_major, subsample_minor;
    bool exact_subsample=false;
    unsigned int num_output_boxes;
    unsigned int num_classes;
    unsigned int threads;
    unsigned int input_channels;
    vector<float> confidence_thresholds;
    bool enable_gpu=false;
    bool output_scale=false;
    int skip_input_channel=-1;

    struct CopyConfig make_copy_config();

    public:

    TFLiteDetector(string&, Json::Value&);
    vector<msg_ImageObject> detect(unordered_map<string, shared_ptr<VisionStage>>&);
    void configure(Json::Value);
};
