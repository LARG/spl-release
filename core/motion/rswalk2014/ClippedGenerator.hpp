#pragma once

#include "Generator.hpp"
#include <motion/RSWalkParameters.h>


class ClippedGenerator : Generator {
   public:
      explicit ClippedGenerator(Generator* g);
      ~ClippedGenerator();
      virtual JointValues makeJoints(ActionCommand::All* request,
                                     Odometry* odometry,
                                     const SensorValues &sensors,
                                     BodyModel &bodyModel,
                                     float ballX,
                                     float ballY);
      virtual bool isActive();
      bool isStanding();
      bool isWalkActive();
      bool isLinedUp();
      void resetLinedUp();
      void reset();
      void readOptions(std::string path);
      void setWalkParameters(const RSWalkParameters &params);

      Generator* generator;
      JointValues old_j;
      bool old_exists;
};
