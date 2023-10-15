#ifndef ACTION_COMMAND
#define ACTION_COMMAND

#include "walk_msg/msg/walk.hpp"

struct ActionCommand
{
    enum ActionType {
        NONE = 0,
        STAND,
        WALK,
        TURN_DRIBBLE,
        GETUP_FRONT,
        GETUP_BACK,
        TIP_OVER,
        KICK,
        INITIAL,
        DEAD,
        REF_PICKUP,
        GOALIE_SIT,
        GOALIE_DIVE_RIGHT,
        GOALIE_DIVE_LEFT,
        GOALIE_CENTRE,
        GOALIE_UNCENTRE,
        GOALIE_INITIAL,
        GOALIE_AFTERSIT_INITIAL,
        DEFENDER_CENTRE,
        GOALIE_FAST_SIT,
        MOTION_CALIBRATE,
        STAND_STRAIGHT,
        LINE_UP,
        TEST_ARMS,
        UKEMI_FRONT,
        UKEMI_BACK,
        GOALIE_STAND,
        NUM_ACTION_TYPES
    };
    ActionType actionType;

    ActionCommand():actionType(WALK), leftArmBehind(false), rightArmBehind(false){}

    // Walk/Kick Parameters
    int forward; // How far forward (negative for backwards)  (mm)
    int left;  // How far to the left (negative for rightwards) (mm)
    float turn; // How much anti-clockwise turn (negative for clockwise) (rad)
    float power; // How much kick power (0.0-1.0)
    float bend;
    float speed;
    float foot;
    bool kick;
    bool enabled;

    bool blocking;
    bool leftArmBehind;
    bool rightArmBehind;

    void make_from_walk_command(walk_msg::msg::Walk::SharedPtr walk_command);
};


#endif
