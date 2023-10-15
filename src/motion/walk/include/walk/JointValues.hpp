#ifndef _JOINT_VALUES
#define _JOINT_VALUES

#include "bodyV6.hpp"

using namespace Sensors;

struct JointValues 
{
   JointValues() 
   {
      for (int i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) 
      {
         angles[i] = NAN;
         stiffnesses[i] = NAN;
         temperatures[i] = NAN;
         currents[i] = NAN;
      }
   }
   JointValues(bool zero) 
   {
      for (int i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) 
      {
         angles[i] = 0;
         stiffnesses[i] = 0;
         temperatures[i] = 0;
         currents[i] = 0;
      }
   }
   /* Angles in radians. Used both for sensor reading and actuating */
   float angles[Joints::NUMBER_OF_JOINTS];
   /* Stiffnesses [-1.0, 0.0..1.0]. Used only for actuating */
   float stiffnesses[Joints::NUMBER_OF_JOINTS];
   /* Temperatures (estimated) in degrees Celcius. Used only for reading */
   float temperatures[Joints::NUMBER_OF_JOINTS];
   float currents[Joints::NUMBER_OF_JOINTS];
};


#endif