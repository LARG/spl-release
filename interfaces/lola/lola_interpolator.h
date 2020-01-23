#pragma once
#include <vector>

/* This class is used to interpolate between initial and target joint commands,
 * in particular joint angles and joint stiffnesses.
 */

#include <memory/JointCommandBlock.h>
#include <memory/JointBlock.h>
#include <stdio.h>

class LolaInterpolator {
    public:
        LolaInterpolator(int firstIndex, int lastIndex, bool log=false, bool cropVals=true);
        void nextMovement(std::array<float, NUM_JOINTS>& targets, std::array<float, NUM_JOINTS>& current, bool send, float interp_time, std::vector<float>& interpolated);
    private:
        bool accepting;  // Accepting if done with last interpolation period.
        float currTime;  // Current amount of time left for interpolation (seconds).
        float totalTime;  // Total amount of time for interpolation (seconds).
        // Index of the first element in the "interpolated" array that we are
        // interpolating over.
        int firstIndex;
        // Index of the last element in the "interpolated" array that we are
        // interpolating over.
        int lastIndex;
        // Initial values for the items we are interpolating.
        std::vector<float> initial;
        // Final values for the items we are interpolating.
        std::vector<float> target;
        // For logging purposes.
        FILE* ofile;
        bool log;
        bool cropVals;
};
