#ifndef _AGENT_UKF
#define _AGENT_UKF

#include <stdint.h>
#include <vector>
#include "localization/ukf.hpp"
#include "localization/measurement.h"

const uint8_t AGENT_STATE_SIZE = 15;

class AgentUkf: public Ukf {
 public:
  AgentUkf();
  ~AgentUkf();

  enum StateMembers {
    StateMemberX = 0,
    StateMemberY,
    StateMemberZ,
    StateMemberRoll,
    StateMemberPitch,
    StateMemberYaw,
    StateMemberVx,
    StateMemberVy,
    StateMemberVz,
    StateMemberVroll,
    StateMemberVpitch,
    StateMemberVyaw,
    StateMemberAx,
    StateMemberAy,
    StateMemberAz
  };

 protected:
  void set_process_noise_covariance();
  void set_transfer_function(const double delta_sec);
  void clampValues(Eigen::VectorXd *state, std::vector<size_t> update_indices);
  void clampStateValues();
  double clampRotation(const double rotation) const;
};

#endif
