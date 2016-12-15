#pragma once

#include <stdio.h>
#include <vision/ml/Typedefs.h>

class Svm {
  public:
    static constexpr int PositiveLabel = 1;
    static constexpr int NegativeLabel = -1;
    virtual bool train(const std::vector<FVec>& features, const std::vector<int>& labels) = 0;
    virtual float predict(const FVec& feature) const = 0;
    virtual void save(std::string file) const = 0;
    virtual void load(std::string file) = 0;
    virtual ~Svm() = default;
};
