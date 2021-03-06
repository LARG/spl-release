#pragma once

#include <cmath>

struct point_3d {
    float x;
    float y;
    float z;

    point_3d() : x(0), y(0), z(0) {}
    point_3d(const point_3d&) = default;
    point_3d(float x, float y, float z) : x(x), y(y), z(z) {}

    float norm() const {
        return std::sqrt(x*x+y*y+z*z);
    }

    float norm_sqr() const {
        return x*x+y*y+z*z;
    }

    void normalize() {
        float in = 1.f / norm();
        x *= in;
        y *= in;
        z *= in;
    }
};

struct point_2d {
    float x;
    float y;

    point_2d() : x(0), y(0) {}
    point_2d(const point_2d&) = default;
    point_2d(float x, float y) : x(x), y(y) {}

    float norm() const {
        return std::sqrt(x*x+y*y);
    }

    float norm_sqr() const {
        return x*x+y*y;
    }

    void normalize() {
        float in = 1.f / norm();
        x *= in;
        y *= in;
    }
};
