#ifndef LIKELIHOOD_ESTIMATOR_H
#define LIKELIHOOD_ESTIMATOR_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <array>
#include <stdarg.h>
#include <vision/Logging.h>

/// @ingroup vision
#define normpdf(x,u,E,k) ( pow(2 * M_PI, -k / 2.0) * 1.0 / sqrt(E.determinant()) * exp(-.5 *\
      (float)((x - u).transpose() * E.inverse() * (x - u))\
      )\
    )

/// @ingroup vision
template<int N>
class LikelihoodEstimator {
  private:
    typedef Eigen::Matrix<float, N, 1> Vector;
    typedef Eigen::Matrix<float, N, N> Matrix;
    float prob_;
    Vector mean_, measurement_, stddev_;
    Matrix cov_;

  protected:
    std::array<std::string, N> names_;
    virtual void setNames() = 0;

  public:
    virtual ~LikelihoodEstimator() = default;
    template<typename... Ts>
    void setMean(Ts... ts) {
      static_assert(sizeof...(Ts) == N, "Invalid number of arguments passed to LikelihoodEstimator::setMean");
      std::array<float,N> data { ts... };
      mean_ = Vector(data.data());
    }
    template<typename... Ts>
    void setStdDev(Ts... ts) {
      static_assert(sizeof...(Ts) == N, "Invalid number of arguments passed to LikelihoodEstimator::setStdDev");
      std::array<float,N> data { ts... };
      Vector v(data.data());
      for(int i = 0; i < N; i++) v[i] *= v[i];
      cov_ = v.asDiagonal();
    }
    template<typename... Ts>
    float getLikelihood(Ts... ts) {
      static_assert(sizeof...(Ts) == N, "Invalid number of arguments passed to LikelihoodEstimator::getLikelihood");
      std::array<float,N> data { ts... };
      measurement_ = Vector(data.data());
      float base = normpdf(mean_, mean_, cov_, N);
      prob_ = normpdf(measurement_, mean_, cov_, N);
      prob_ /= base;
      if(std::isnan(prob_)) prob_ = 0.0;
      return prob_;
    }
    void printLast() {
      for(unsigned int i = 0; i < names_.size(); i++) {
        printf("%s: %05.2f (u=%2.1f,s=%2.1f),", names_[i].c_str(), measurement_[i], mean_[i], stddev_[i]);
      }
      printf("p: %.2f\n", prob_);
    }

    inline void logLast(int num, TextLogger *textlogger) {
      if(!ALLOW_DEBUG_LOG) return;
      for(unsigned int i = 0; i < names_.size(); i++) {
        tlog(num,"%s: %05.2f,", names_[i].c_str(), measurement_[i]);
      }
      tlog(num,"p: %.2f", prob_);
    }
};

#endif

