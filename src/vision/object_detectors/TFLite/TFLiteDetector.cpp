#include<TFLiteDetector.hpp>

using namespace tflite;
using namespace evaluation;


#define TFLITE_MINIMAL_CHECK(x)                              \
  if (!(x))                                                  \
  {                                                          \
    fprintf(stderr, "Error at %s:%d\n", __FILE__, __LINE__); \
    exit(1);                                                 \
  }


enum PointInRectangle {XMIN, YMIN, XMAX, YMAX};

vector<float> GetPointFromXYWH(const vector<vector<float>> & xywh,
                              const PointInRectangle & pos);
vector<float> ComputeArea(const vector<float> & x1,
                          const vector<float> & y1,
                          const vector<float> & x2,
                          const vector<float> & y2);
template <typename T> vector<int> argsort(const vector<T> & v);
vector<float> Maximum(const float & num,
                      const vector<float> & vec);
vector<float> Minimum(const float & num,
                      const vector<float> & vec);
vector<float> CopyByIndexes(const vector<float> & vec,
                            const vector<int> & idxs);
vector<int> RemoveLast(const vector<int> & vec);
vector<float> Subtract(const vector<float> & vec1,
                       const vector<float> & vec2);
vector<float> Multiply(const vector<float> & vec1,
		                   const vector<float> & vec2);
vector<float> Divide(const vector<float> & vec1,
		                 const vector<float> & vec2);
vector<int> WhereLarger(const vector<float> & vec,
                        const float & threshold);
vector<int> RemoveByIndexes(const vector<int> & vec,
                            const vector<int> & idxs);
template <typename T>vector<T> FilterVector(const vector<T> & vec,
    const vector<int> & idxs);
vector<int> nms(
    const vector<vector<float>> & boxes,
    const float & threshold);

#define vector_append(dest, src) {dest.insert(dest.end(), src.begin(), src.end());}

TFLiteDetector::TFLiteDetector(string& name_, Json::Value& config) : ObjectDetector(name_, config)
{
    configure(config);
}


void TFLiteDetector::configure(Json::Value config)
{
    ObjectDetector::configure(config);
    ReadJSONAttribute(config,model_path,String);
    ReadJSONAttribute(config,input_width,Int);
    ReadJSONAttribute(config,input_height,Int);
    ReadJSONAttribute(config,model_input_width,Int);
    ReadJSONAttribute(config,model_input_height,Int);
    ReadJSONAttribute(config,num_classes,Int);
    ReadJSONAttribute(config,num_output_boxes,Int);
    ReadJSONAttribute(config,subsample_major,Int);
    ReadJSONAttribute(config,subsample_minor,Int);
    ReadJSONAttribute(config,threads,Int);
    ReadJSONAttribute(config,enable_gpu,Bool);
    ReadJSONAttribute(config,output_scale,Bool);
    ReadJSONAttribute(config,exact_subsample,Bool);
    ReadJSONAttribute(config,input_channels,Int);
    ReadJSONAttribute(config,skip_input_channel,Int);

    // Copy list of per-class confidence thresholds
    confidence_thresholds.clear();
    for(auto conf_thr: config["confidence_thresholds"])
      confidence_thresholds.push_back(conf_thr.asFloat());

    // Load model
    model = FlatBufferModel::BuildFromFile(model_path.c_str());
    if((model) == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load model file: %s", model_path.c_str());
        exit(1);
    }

    // Build the interpreter with the InterpreterBuilder.
    InterpreterBuilder builder(*(model), resolver);
    builder(&(interpreter), 1);
    TFLITE_MINIMAL_CHECK((interpreter) != nullptr);

    // Create the delegate object using the helpers provided in TFLite.
    if (enable_gpu)
    {
        std::cout << "Initializing GPU Delegate\n";
        TfLiteGpuDelegateOptionsV2 options = TfLiteGpuDelegateOptionsV2Default();
        options.inference_priority1 = TFLITE_GPU_INFERENCE_PRIORITY_MIN_LATENCY;
        options.inference_priority2 = TFLITE_GPU_INFERENCE_PRIORITY_AUTO;
        options.inference_priority3 = TFLITE_GPU_INFERENCE_PRIORITY_AUTO;
        options.inference_preference =
            TFLITE_GPU_INFERENCE_PREFERENCE_SUSTAINED_SPEED;
        delegate = (CreateGPUDelegate(&options));
        if (!(delegate)) {
            std::cerr << "GPU acceleration is unsupported on this platform.";
            exit(1);
        } else {
            std::cout << "GPU delegate created\n";
        }
    }
    else
    {
        std::cout << "Initializing XNNPack Delegate\n";
        delegate = (CreateXNNPACKDelegate(threads));
        if (!(delegate)) {
            std::cerr << "XNNPACK acceleration is unsupported on this platform.";
            exit(1);
        } else {
            std::cout << "XNNPACK delegate created\n";
        }
    }

    // This step is necessary to make it use the delegate. Not clear
    // if it can be performed later or it has to be done now.
    // After this, everything is as usual.
    if (interpreter->ModifyGraphWithDelegate(delegate.get()) !=
        kTfLiteOk) {
        std::cerr << "Failed to apply " << " delegate.\n";
        exit(-1);
    } else {
        std::cout << "Successfully Applied " << " delegate.\n";
    }
    
    // Allocate tensor buffers.
    TFLITE_MINIMAL_CHECK(interpreter->AllocateTensors() == kTfLiteOk);
    TFLITE_MINIMAL_CHECK(interpreter->Invoke() == kTfLiteOk);
    cout << "Init inference test... ok." << endl;
}


struct CopyConfig TFLiteDetector::make_copy_config()
{
  struct CopyConfig cfg = {
    input_channels,
    skip_input_channel,
    subsample_major,
    subsample_minor
  };
  return cfg;
}

vector<msg_ImageObject> TFLiteDetector::detect(unordered_map<string, shared_ptr<VisionStage>>& stages)
{
  auto image_source = dynamic_pointer_cast<ImageSource>(stages[image_source_name]);
  auto copy_cfg = make_copy_config();

  bool roi_detection = invoke_cfg.isMember("detect_center");

  if(roi_detection)
  {
      auto center_v = invoke_cfg["detect_center"];
      if(center_v.size() != 2)
      {
          RCLCPP_ERROR(this->get_logger(), "TFLiteDetector '%s': Wrong number of values for ROI center_v point. Expected 2, got %d", stage_name.c_str(), center_v.size());
          exit(1);
      }
      int i = 0;
      Vec2f center;
      for(auto v : center_v)
      {
          center[i++] = v.asFloat();
      }
      image_source -> update_copy_config_roi(copy_cfg, center, Vec2i(model_input_width, model_input_height));
  }
  
  float * input_tensor = ((interpreter))->typed_input_tensor<float>(0);
  // RCLCPP_INFO(this->get_logger(), "Copy range: %d %d %d %d", copy_cfg.rect[0], copy_cfg.rect[1], copy_cfg.rect[2], copy_cfg.rect[3]);

  // Copy image with subsampling
  image_source->copy_to_buffer<float>(input_tensor, copy_cfg);

  // RCLCPP_INFO(this->get_logger(), "Copy range: %d %d %d %d", copy_cfg.rect[0], copy_cfg.rect[1], copy_cfg.rect[2], copy_cfg.rect[3]);
  // copy_cfg.skip_input_channel = -1;
  // auto im_roi = image_source->copy_to_mat(copy_cfg);
  // Mat im_roi_bgr;
  // cvtColor(im_roi, im_roi_bgr, COLOR_YUV2BGR_YUYV);
  // imshow("im_roi", im_roi_bgr);
  // waitKey(1);

  // Run inference
  std::chrono::high_resolution_clock::time_point model_begin = std::chrono::high_resolution_clock::now();
  TFLITE_MINIMAL_CHECK(((interpreter))->Invoke() == kTfLiteOk);
  std::chrono::high_resolution_clock::time_point model_end = std::chrono::high_resolution_clock::now();
  RCLCPP_DEBUG(this->get_logger(), "Model inference time %.1f ms", stage_name.c_str(),
                std::chrono::duration_cast<std::chrono::microseconds>(model_end - model_begin).count()/1000.0);

  // Read output tensor
  float wr=1.0, hr=1.0;
  if(output_scale)
  {
    wr = 1.0/(model_input_width);
    hr = 1.0/(model_input_height);
  }

  // Process outputs for each class
  vector<vector<vector<float>>> boxes(MAX_OBJECT_CLASSES);
  vector<vector<float>> confidence(MAX_OBJECT_CLASSES);
  vector<vector<int>> cls(MAX_OBJECT_CLASSES);
  
  
  int num_found_boxes = 0;

  float * output_tensor = (interpreter)->typed_output_tensor<float>(0);

  for (auto i = 0; i < num_output_boxes; ++i) {
    int ix = (5 + num_classes) * i;
    float conf = output_tensor[ix + 4];
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

    float min_confidence = confidence_thresholds[best_class];

    if (conf < min_confidence)
      continue;

    confidence[best_class].push_back(conf);
    cls[best_class].push_back(best_class);

    vector<float> box = {
      wr * output_tensor[ix++],
      hr * output_tensor[ix++],
      wr * output_tensor[ix++],
      hr * output_tensor[ix++]
    };
    boxes[best_class].push_back(box);
    num_found_boxes++;
  }
  
  // printf("Extracted output tensor data\n");
  // printf("Found %d boxes above min confidence.\n", num_found_boxes);
  RCLCPP_DEBUG(this->get_logger(), "Found %d boxes above min confidence.\n", num_found_boxes);

  // // class agnostic non-max spression
  // const float threshold = 0.5;
  vector<vector<float>> fboxes;
  vector<float> fconfidence;
  vector<int> fcls;

  for(int c=0; c<MAX_OBJECT_CLASSES; c++)
  {
    vector<int> pick = nms(boxes[c], confidence_thresholds[c]);
    vector<vector<float>> fboxes_ = FilterVector(boxes[c], pick);
    vector<float> fconfidence_ = FilterVector(confidence[c], pick);
    vector<int> fcls_ = FilterVector(cls[c], pick);

    vector_append(fboxes, fboxes_);
    vector_append(fconfidence, fconfidence_);
    vector_append(fcls, fcls_);
  }

  RCLCPP_DEBUG(this->get_logger(), "%d boxes after NMS.\n", fboxes.size());

  float max_conf = 0;
  vector<msg_ImageObject> classwise_best_objects(MAX_OBJECT_CLASSES, msg_ImageObject());
  unordered_set<int> classes_found;
  // printf("Found %d boxes above min confidence after nms.\n", num_found_boxes_post_nms);
  vector<msg_ImageObject> detected_objects;
  for (auto i = 0; i < fboxes.size(); i++) {
    auto & box = fboxes[i];
    auto rect = msg_ImageObject();
    rect.object_type = object_map[fcls[i]];
    // // TODO : Don't hardcode
    // switch (fcls[i])
    // {
    // case 0: // ball
    //   rect.object_type = IO_BALL;
    //   break;
    // case 1: // cross
    //   rect.object_type = IO_CROSS;
    //   break;
    // case 2: // robot
    //   rect.object_type = IO_ROBOT;
    //   break;
    // default:
    //   break;
    // }

    rect.center_x = box[0];
    rect.center_y = box[1];
    rect.size_x = box[2];
    rect.size_y = box[3];

    if(roi_detection)
    {
      Vec2f img_size_f(image_source->get_image_size());
      // Normalized ROI shift and scale
      Vec2f roi_shift_norm = Vec2f(copy_cfg.rect[0], copy_cfg.rect[1]).mul(Vec2f(1.0/img_size_f[0],1.0/img_size_f[1]));
      Vec2f roi_scale = Vec2f(model_input_width, model_input_height).mul(Vec2f(1.0/img_size_f[0],1.0/img_size_f[1]))*copy_cfg.get_scale_factor();

      rect.center_x = rect.center_x * roi_scale[0] + roi_shift_norm[0];
      rect.center_y = rect.center_y * roi_scale[1] + roi_shift_norm[1];
      rect.size_x = rect.size_x * roi_scale[0];
      rect.size_y = rect.size_y * roi_scale[1];
    }

    rect.confidence = fconfidence[i];

    // Store the highest confidence objects of each class
    if(classes_found.find(fcls[i])==classes_found.end() || rect.confidence > classwise_best_objects[fcls[i]].confidence)
      classwise_best_objects[fcls[i]] = rect;

    classes_found.insert(fcls[i]);
  }

  for (auto c: classes_found)
    if(classwise_best_objects[c].confidence<=1.0)
      detected_objects.push_back(classwise_best_objects[c]);

  return detected_objects;
}


// ===================================
// NMS function definitions start here
// ===================================

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
