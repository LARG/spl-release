#pragma once
/* The robot contains four sensors on each foot -- two on front and two in back */

struct Foot {
    float fl = 0.f;  // front left
    float fr = 0.f;  // front right
    float rl = 0.f;  // rear left
    float rr = 0.f;  // rear right
};

struct FSR {
    Foot left;
    Foot right;
};
