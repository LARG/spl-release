#ifndef GAMESTATE_
#define GAMESTATE_

#include <memory/MemoryBlock.h>
#include <common/States.h>
#include <ctime>
#include <schema/gen/GameStateBlock_generated.h>

DECLARE_INTERNAL_SCHEMA(struct GameStateBlock : public MemoryBlock {
  public:
    SCHEMA_METHODS(GameStateBlock);
    GameStateBlock():
      isPenaltyKick(false)
    {
      header.version = 2;
      header.size = sizeof(GameStateBlock);


      prevstate_ = state_ = INITIAL;
      ourKickOff = true;
      gamePhase = PHASE_NORMAL;
      isFirstHalf = 1;
      gameContTeamNum=1;
      ourScore = 0;
      opponentScore = 0;
      secsTillUnpenalised = 0;
      secsRemaining = 600;
      messageBudget = 0;
      frameReceived = 0;
      lastStateChangeFromButton = false;
      lastTimeLeftPenalized = -1;
      whistleTime =  0;
      isFreeKick = false;
      isFreeKickTypeGoal = false;
      isFreeKickTypePenalty = false;
      spawnFromPenalty=false;
    }

    State state() { return state_; }
    State prevstate() { return prevstate_; }
    bool change() { return stateElapsedTime() == 0; }
    void setState(State state);
    int stateElapsedTime() {
      return time(NULL) - stateStartTime_;
    }
    int stateStartTime() {
      return stateStartTime_;
    }
    void whistleOverride(State state);
    int whistleElapsedTime() { return time(NULL) - whistleTime; }

    bool spawned_whistle_positions_ = false;

    // Our team number
    SCHEMA_FIELD(int gameContTeamNum);
    // Whether we are in the penalty shootout phase
    SCHEMA_FIELD(bool ourKickOff);
    // Seconds remaining in the half
    SCHEMA_FIELD(int secsRemaining);
    // Number of goals we've scored
    SCHEMA_FIELD(int ourScore);
    // Number of goals the opponents have scored
    SCHEMA_FIELD(int opponentScore);
    // Seconds left in penalty for this robot
    SCHEMA_FIELD(int secsTillUnpenalised);
    // One of normal, penaltyshoot, overtime, or timeout
    SCHEMA_FIELD(Phase gamePhase);
    // Whether we are in the penaltyshoot phase (should be true whenever
    // gamePhase == PHASE_PENALTYSHOOT)
    SCHEMA_FIELD(bool isPenaltyKick);
    // 1 in first half, 0 otherwise
    SCHEMA_FIELD(int isFirstHalf);
    // number of remaining messages our team can send
    SCHEMA_FIELD(int messageBudget);
    // frame number of most recent message from GC
    SCHEMA_FIELD(int frameReceived);
    // time last whistle was heard
    SCHEMA_FIELD(int whistleTime);
    // whether we are in a FreeKick
    SCHEMA_FIELD(bool isFreeKick);
    // whether the type of FreeKick is GoalFreeKick-like
    SCHEMA_FIELD(bool isFreeKickTypeGoal);
    // whether the type of FreeKick is a PenaltyKick
    SCHEMA_FIELD(bool isFreeKickTypePenalty);
    // whether the last state change was from a button press
    SCHEMA_FIELD(bool lastStateChangeFromButton);
    // unused (??)
    SCHEMA_FIELD(float lastTimeLeftPenalized);
    // whether the robot is moved back to penalized area for penalty
    SCHEMA_FIELD(bool spawnFromPenalty);
  private:
    // current game state (READY, PLAYING, etc.)
    SCHEMA_FIELD(State state_);
    // previous game state
    SCHEMA_FIELD(State prevstate_);
    // time since state began
    SCHEMA_FIELD(int stateStartTime_);
});

#endif 
