#include <memory/GameStateBlock.h>
#include <common/tinyformat.h>

void GameStateBlock::setState(State state) {
  // If we've heard the whistle recently don't go back into set
  // This is because we ignore the state communicated by GameController 
  // and go into PLAYING if we hear the whistle in SET.
  // GC gives state PLAYING 20 seconds after whistle is heard
  if(whistleElapsedTime() <= 20 && state == SET && state_ == PLAYING) {
    return;
  }
  // Same as above, but transitioning from PLAYING to READY
  if(whistleElapsedTime() <= 20 && state == PLAYING && state_ == READY) {
    return;
  }
  // If in UNSTIFF, do not allow any transition except to INITIAL
  if(state != INITIAL and state_ == UNSTIFF and lastStateChangeFromButton == true) {
      return;
  }

  // Otherwise proceed normally
  if(state != state_) {
    stateStartTime_ = time(NULL);
    prevstate_ = state_;
    state_ = state;
  }
  if(state_ == SET) {
    spawned_whistle_positions_ = false;
  }
}

void GameStateBlock::whistleOverride(State state) {
  whistleTime = time(NULL);
  setState(state);
}
