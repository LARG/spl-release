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
  CvSVMParams params;
  params.svm_type = CvSVM::NU_SVC;
  params.kernel_type = CvSVM::LINEAR;
  params.term_crit = cv::TermCriteria(cv::TermCriteria::MAX_ITER, 10'000, 0);
  params.nu = 0.01;
  params.degree = 3;
  return _cvsvm.train_auto(cvfeatures, cvlabels, cv::Mat(), cv::Mat(), params);
}

float OcvSvm::predict(const FVec& feature) const {
  FVec cvfeature(1, feature.rows);
  for(int i = 0; i < feature.rows; i++)
    cvfeature(0,i) = feature(i,0);
  float response = _cvsvm.predict(cvfeature);
  return response;
}

void OcvSvm::save(std::string file) const {
  _cvsvm.save(file.c_str());
}

void OcvSvm::load(std::string file) {
  _cvsvm.load(file.c_str());
}

