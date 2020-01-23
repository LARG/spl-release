#include <vision/ml/OcvSvm.h>

bool OcvSvm::train(const std::vector<FVec>& features,const std::vector<int>& labels) {

  assert(features.size() > 0 && labels.size() == features.size());
  cv::Mat cvlabels(labels.size(), 1, CV_32FC1), cvfeatures(features.size(), features[0].rows, CV_32FC1);
  float max = 0;
  for(int i = 0; i < features.size(); i++) {
    for(int j = 0; j < features[i].rows; j++) {
      cvfeatures.at<float>(i,j) = features[i](j,0);
      if(max < features[i](j,0))
        max = features[i](j,0);
    }
    cvlabels.at<float>(i,0) = labels[i];
  }
#ifdef USE_OPENCV2  
  CvSVMParams params;
  params.svm_type = CvSVM::NU_SVC;
  params.kernel_type = CvSVM::LINEAR;
  params.term_crit = cv::TermCriteria(cv::TermCriteria::MAX_ITER, 10'000, 0);
  params.nu = 0.01;
  params.degree = 3;
  return _cvsvm.train_auto(cvfeatures, cvlabels, cv::Mat(), cv::Mat(), params);

#else  
 
  cv::Ptr<cv::ml::TrainData> trainData = cv::ml::TrainData::create(cvfeatures, cv::ml::ROW_SAMPLE, cvlabels);
  _cvsvm = cv::ml::SVM::create();
  _cvsvm->setType(cv::ml::SVM::NU_SVC);
  _cvsvm->setKernel(cv::ml::SVM::LINEAR);
  _cvsvm->setNu(0.01);
  _cvsvm->setDegree(3);
  _cvsvm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 10'000, 0));
  _cvsvm->trainAuto(trainData);
  return _cvsvm;
  
#endif  
}

float OcvSvm::predict(const FVec& feature) const {


  FVec cvfeature(1, feature.rows);
  for(int i = 0; i < feature.rows; i++)
    cvfeature(0,i) = feature(i,0);
#ifdef USE_OPENCV2  
  float response = _cvsvm.predict(cvfeature);
#else
  float response = _cvsvm->predict(cvfeature);
#endif
  return response;

}

void OcvSvm::save(std::string file) const {
#ifdef USE_OPENCV2
  _cvsvm.save(file.c_str());
#else
  _cvsvm->save(file);
#endif
}

void OcvSvm::load(std::string file) {
#ifdef USE_OPENCV2
  _cvsvm.load(file.c_str());
#else
  _cvsvm = cv::ml::SVM::load<cv::ml::SVM>(file);
#endif
}

