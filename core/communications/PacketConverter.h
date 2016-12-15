#pragma once

#include <communications/SPLCoachMessage.h>
#include <common/TeamPacket.h>
#include <common/CoachPacket.h>

typedef TeamPacket TeamPacket;
typedef CoachPacket UTCoachMessage;

/// @ingroup communications
class PacketConverter {
  public:
    template<typename T>
    static T convert(TeamPacket message) {
      return T();
    }

    static TeamPacket convert(SPLStandardMessage& message) {
      return TeamPacket();
    }

    static UTCoachMessage convert(SPLCoachMessage message) {
      return UTCoachMessage();
    }

    static SPLCoachMessage convert(UTCoachMessage message) {
      return SPLCoachMessage();
    }
};
