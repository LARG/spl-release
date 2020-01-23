#pragma once

/* LEDs present on the robot are turned on/off by LoLA */

#include <array>

struct RGB {
    float r, g, b;
    static const RGB RED;
    static const RGB GREEN;
    static const RGB BLUE;
};

using Eye = std::array<RGB, 8>;

struct Eyes {
    Eye left;
    Eye right;
};

struct Ears {
    std::array<float, 10> left;
    std::array<float, 10> right;
};

using SkullSide = std::array<float, 6>;

struct Skull {
    SkullSide left;
    SkullSide right;
};

struct Leds {
    Ears ears;
    Eyes eyes;
    RGB chest;
    RGB left_foot;
    RGB right_foot;
    Skull skull;
};

template<typename RGB>
void set_led(float r, float g, float b, RGB* leds) {
    (*leds).r = r;
    (*leds).g = g;
    (*leds).b = b;
}
