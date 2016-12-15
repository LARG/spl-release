#include <vision/ml/Classifier.h>

constexpr int Classifier::Nothing;

Classifier::Classifier(const std::string& name, const std::string& directory) 
  : name_(name), directory_(directory) {
}
