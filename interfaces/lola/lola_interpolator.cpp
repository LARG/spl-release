#include "lola_interpolator.h"
#include <cmath>
#include <math/Geometry.h>
#include <memory/JointBlock.h>

LolaInterpolator::LolaInterpolator(int firstIndex, int lastIndex, bool log, bool cropVals) :
    firstIndex(firstIndex),
    lastIndex(lastIndex),
    accepting(true),
    currTime(-1),
    totalTime(-1),
    initial(lastIndex - firstIndex + 1, 0),
    target(lastIndex - firstIndex + 1, 0),
    log(log),
    cropVals(cropVals)
{
    if(log) {
        remove("log.txt");
    }
}

void LolaInterpolator::nextMovement(
        // Commands from higher level code.
        std::array<float, NUM_JOINTS>& inputTarget,
        // Current values.
        std::array<float, NUM_JOINTS>& inputCurrent,
        // Signal about whether the intput angles should be sent for
        // interpolation.
        bool send,
        // Time over which to interpolate.
        float interp_time,
        // Results are stored here.
        std::vector<float>& interpolated
) {
    // ms -> seconds.
    interp_time /= 1000.0;
    
    int numJoints = lastIndex - firstIndex + 1;

    // We do not accept target values while in the middle of another
    // interpolation period.
    if(this->accepting) {
        this->accepting = false;
        // Sometimes the higher level code is not ready to send commands.
        // See "send_body_angles_" in core/memory/JointCommandBlock.h.
        if(send) {
            this->totalTime = interp_time;
            this->currTime = totalTime;
            for(int i = 0; i < numJoints; i++) {
                int inIndex = i + firstIndex;
                this->target[i] = inputTarget[inIndex];
                // Cropping between permissible levels.
                if(cropVals) {
                    this->target[i] = crop(this->target[i], minJointLimits[inIndex], maxJointLimits[inIndex]);
                }
            }
        } else {
            // If we are not ready to send new joint commands but yet the 
            // current interpolation period has expired, maintain the target
            // from the last interpolation period for one more timestep.
            this->totalTime = 0.001;
            this->currTime = 0.001;
       }
    }

    this->currTime -= 0.012;  // 83 Hz.
    if(this->currTime <= 0.0) {
        this->accepting = true;
        for(int i = 0; i < numJoints; ++i) {
            this->initial[i] = this->target[i];
        }
    }

    // sin interpolation
    float interp_factor = sin(M_PI/2 * (totalTime - currTime)/totalTime);
 
    for(int i = 0; i < numJoints; ++i) {
        int inIndex = i + firstIndex;
        interpolated[inIndex] = initial[i] + interp_factor * (target[i] - initial[i]);
    }
 
    if(log) {
        FILE* ofile = fopen("log.txt", "a+");
        fprintf(ofile, "%f\n", inputTarget[LKneePitch]);
        fprintf(ofile, "%f\n", inputCurrent[LKneePitch]);
        fprintf(ofile, "%f\n", interpolated[LKneePitch]);
        fclose(ofile);
    }
}
