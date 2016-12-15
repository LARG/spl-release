#pragma once

enum FeatureExtractors : int {
  Undefined = 0,
  SIFT = 1,
  QuantizedSIFT = 2,
  HOG = 4,
  QuantizedHOG = 8,
  Raw = 16
};
