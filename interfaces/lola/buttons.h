#pragma once
/* Classes to store button presses on the robot. */

// The "toes" of the robot has two sensors on each foot.
struct Bumper {
    float left;
    float right;
};

// The top of the head has three touch sensors.
struct HeadTouch {
    float front;
    float middle;
    float rear;
};

struct HandTouch {
    float back;
    float left;
    float right;
};

struct Buttons {
    Bumper left_foot;
    Bumper right_foot;
    float chest;
    HeadTouch head;
    HandTouch left_hand;
    HandTouch right_hand;
};
