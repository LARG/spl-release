#ifndef _UKF
#define _UKF

#include <Eigen/Dense>
#include <vector>
#include "localization/measurement.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class Ukf {
 public:
  explicit Ukf(int size);
  ~Ukf();

  void initialize(double alpha = 0.001, double kappa = 1, double beta = 2);
  void predict(double delta_sec);
  void update(const Measurement & measurement);
  void setState(VectorXd state) { state_ = state; }
  void setCovariance(MatrixXd covariance) {
    estimate_error_covariance_ = covariance;
  }
  VectorXd getState() const { return state_; }
  MatrixXd getCovariance() const { return estimate_error_covariance_; }


 protected:
  double alpha;
  double beta;
  double kappa;
  double lambda_;
  int state_size;
  bool uncorrected_;
  VectorXd state_;
  std::vector<Eigen::VectorXd> sigma_points_;
  MatrixXd weighted_covar_sqrt_;
  MatrixXd transfer_function_;
  MatrixXd process_noise_covariance_;
  MatrixXd estimate_error_covariance_;
  MatrixXd identity_;
  MatrixXd initial_estimate_covariance_;
  std::vector<double> state_weights_;
  std::vector<double> covar_weights_;
  bool checkMahalanobisThreshold(
    const Eigen::VectorXd & innovation,
    const Eigen::MatrixXd & innovation_covariance, const double n_sigmas);

  virtual void clampValues(VectorXd*, std::vector<size_t>) {}
  virtual void clampStateValues() {}
  virtual void set_process_noise_covariance() {};
  virtual void set_transfer_function(const double delta_sec) = 0;
};

#endif
