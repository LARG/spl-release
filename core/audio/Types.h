#pragma once
#include <Eigen/Core>
#include <math/MatrixOperations.h>

using Spectrum = Eigen::Matrix<float, 1, Eigen::Dynamic>;
using Spectra = Eigen::MatrixXf;
using SpectrumStats = MatrixOperations::Stats<float>;
