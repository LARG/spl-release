#pragma once

#include "point_3d.h"

struct YPR {
    float yaw, pitch, roll;

    YPR():yaw(0),pitch(0),roll(0){}
    YPR(float _yaw,float _pitch,float _roll):yaw(_yaw),pitch(_pitch),roll(_roll){}

    YPR operator-(const YPR &o){
        return YPR(yaw-o.yaw,pitch-o.pitch,roll-o.roll);
    }

    YPR operator+(const YPR &o){
        return YPR(yaw+o.yaw,pitch+o.pitch,roll+o.roll);
    }
};

struct IMU {
    YPR gyr;  // Gyroscope
    point_3d accel;  // Accelerometer
    // Check out http://doc.aldebaran.com/2-1/family/robots/inertial_robot.html#compute-torso-angle-algorithm.
    // These are body angles
    point_2d angles;
};
