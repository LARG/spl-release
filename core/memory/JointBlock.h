#ifndef JOINTBLOCK_
#define JOINTBLOCK_

#include <iostream>
#include <common/RobotInfo.h>
#include <memory/MemoryBlock.h>
#include <schema/gen/JointBlock_generated.h>

DECLARE_INTERNAL_SCHEMA(struct JointBlock : public MemoryBlock {
  public:
    SCHEMA_METHODS(JointBlock);
    JointBlock() {
      header.version = 2;
      header.size = sizeof(JointBlock);
      for (int i=0; i<NUM_JOINTS; i++) {
        values_[i] = 0;
        stiffness_[i] = 0;
        temperature_[i] = 0;
        current_[i] = 0;
        status_[i] = 0;
      }
    }

    float getJointValue(int i) const {
      return values_[i];
    }

    float getJointDelta(int i) const {
      return values_[i] - prevValues_[i];
    }
    
    float getJointTemperature(int i){
      return temperature_[i];
    }

    SCHEMA_FIELD(std::array<float,NUM_JOINTS> prevValues_);
    SCHEMA_FIELD(std::array<float,NUM_JOINTS> values_);
    SCHEMA_FIELD(std::array<float,NUM_JOINTS> stiffness_);
    SCHEMA_FIELD(std::array<float,NUM_JOINTS> temperature_);
    SCHEMA_FIELD(std::array<float,NUM_JOINTS> current_);
    SCHEMA_FIELD(std::array<int,NUM_JOINTS> status_);

});

#endif
