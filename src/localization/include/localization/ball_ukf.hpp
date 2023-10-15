#ifndef _BALL_UKF
#define _BALL_UKF

#include <stdint.h>
#include <vector>
#include "localization/ukf.hpp"

const uint8_t BALL_STATE_SIZE = 4;

class BallUkf: public Ukf {
 public:
  BallUkf();
  ~BallUkf();

  enum StateMembers {
    StateMemberX = 0,
    StateMemberY,
    StateMemberVx,
    StateMemberVy,
  };
 protected:
  void set_process_noise_covariance();
  void set_transfer_function(const double delta_sec);
};

#endif
