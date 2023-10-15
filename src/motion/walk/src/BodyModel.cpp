#include "walk/BodyModel.hpp"
#include "walk/bodyV6.hpp"
#include "walk/basic_math.hpp"
// #include "utils/matrix_helpers.hpp"

BodyModel::BodyModel()
{
   forwardR = forwardL = 0.0;
   isLeftPhase = false;

  //Used to AutoCalibrate Footsensors
	fsLfr = fsLfl = fsLrr = fsLrl = 0.1;                   // keeps maximum foot sensor readings
	fsRfr = fsRfl = fsRrr = fsRrl = 0.1;                   // small value deemed close to zero initially
}

void BodyModel::update(const SensorValues &sensors) {
  // Update maximum sensed foot-sensor readings
	float temp = sensors.sensors[Sensors::LFoot_FSR_FrontLeft]; if(fsLfl<temp and fsLfl<5.0) fsLfl = temp;
	temp = sensors.sensors[Sensors::LFoot_FSR_FrontLeft];       if(fsLfl<temp and fsLfl<5.0) fsLfl = temp;
	temp = sensors.sensors[Sensors::LFoot_FSR_FrontRight];      if(fsLfr<temp and fsLfr<5.0) fsLfr = temp;
	temp = sensors.sensors[Sensors::LFoot_FSR_RearLeft];        if(fsLrl<temp and fsLrl<5.0) fsLrl = temp;
	temp = sensors.sensors[Sensors::LFoot_FSR_RearRight];       if(fsLrr<temp and fsLrr<5.0) fsLrr = temp;
	temp = sensors.sensors[Sensors::RFoot_FSR_FrontLeft];       if(fsRfl<temp and fsRfl<5.0) fsRfl = temp;
	temp = sensors.sensors[Sensors::RFoot_FSR_FrontRight];      if(fsRfr<temp and fsRfr<5.0) fsRfr = temp;
	temp = sensors.sensors[Sensors::RFoot_FSR_RearLeft];        if(fsRrl<temp and fsRrl<5.0) fsRrl = temp;
	temp = sensors.sensors[Sensors::RFoot_FSR_RearRight];       if(fsRrr<temp and fsRrr<5.0) fsRrr = temp;
	lastZMPL = ZMPL;
	ZMPL = 0;
	// Calculate ZMPL (left-right) used to eg switch support foot in Walk2014
	float pressureL =
	  +sensors.sensors[Sensors::LFoot_FSR_FrontLeft]/fsLfl
	  + sensors.sensors[Sensors::LFoot_FSR_FrontRight]/fsLfr
	  + sensors.sensors[Sensors::LFoot_FSR_RearLeft]/fsLrl
	  + sensors.sensors[Sensors::LFoot_FSR_RearRight]/fsLrr;
	float pressureR =
	  +sensors.sensors[Sensors::RFoot_FSR_FrontLeft]/fsRfl
	  + sensors.sensors[Sensors::RFoot_FSR_FrontRight]/fsRfr
	  + sensors.sensors[Sensors::RFoot_FSR_RearLeft]/fsRrl
	  + sensors.sensors[Sensors::RFoot_FSR_RearRight]/fsRrr;
	float totalPressure = pressureL + pressureR;
	if (ABS(totalPressure) > 0.000001f) {
			ZMPL =
			(  .080 * sensors.sensors[Sensors::LFoot_FSR_FrontLeft]/fsLfl
			+ .030 * sensors.sensors[Sensors::LFoot_FSR_FrontRight]/fsLfr
			+ .080 * sensors.sensors[Sensors::LFoot_FSR_RearLeft]/fsLrl
			+ .030 * sensors.sensors[Sensors::LFoot_FSR_RearRight]/fsLrr
			- .030 * sensors.sensors[Sensors::RFoot_FSR_FrontLeft]/fsRfl
			- .080 * sensors.sensors[Sensors::RFoot_FSR_FrontRight]/fsRfr
			- .030 * sensors.sensors[Sensors::RFoot_FSR_RearLeft]/fsRrl
			- .080 * sensors.sensors[Sensors::RFoot_FSR_RearRight]/fsRrr) / totalPressure;
		}
//    processUpdate(sensors);
}

// For disturbance rejection
bool BodyModel::isFootOnGround(const SensorValues &sensors) {
   float leftFrontL = sensors.sensors[Sensors::LFoot_FSR_FrontLeft];
   float leftFrontR = sensors.sensors[Sensors::LFoot_FSR_FrontRight];
   float rightFrontL = sensors.sensors[Sensors::RFoot_FSR_FrontLeft];
   float rightFrontR = sensors.sensors[Sensors::RFoot_FSR_FrontRight];
   float leftRearL  = sensors.sensors[Sensors::LFoot_FSR_RearLeft];
   float leftRearR  = sensors.sensors[Sensors::LFoot_FSR_RearRight];
   float rightRearL = sensors.sensors[Sensors::RFoot_FSR_RearLeft];
   float rightRearR = sensors.sensors[Sensors::RFoot_FSR_RearRight];
   if (isLeftPhase) {
      return rightFrontL > 0.01 && rightFrontR > 0.01 &&
             rightRearL > 0.01 && rightRearR > 0.01;
   }
   return leftFrontL > 0.01 &&  leftFrontR > 0.01 &&
          leftRearL > 0.01 && leftRearR > 0.01;
}

bool BodyModel::isOnFrontOfFoot(const SensorValues &sensors) {
   float leftFront = sensors.sensors[Sensors::LFoot_FSR_FrontLeft] +
                     sensors.sensors[Sensors::LFoot_FSR_FrontRight];
   float rightFront = sensors.sensors[Sensors::RFoot_FSR_FrontLeft] +
                      sensors.sensors[Sensors::RFoot_FSR_FrontRight];
   float leftRear  = sensors.sensors[Sensors::LFoot_FSR_RearLeft] +
                     sensors.sensors[Sensors::LFoot_FSR_RearRight];
   float rightRear = sensors.sensors[Sensors::RFoot_FSR_RearLeft] +
                     sensors.sensors[Sensors::RFoot_FSR_RearRight];
   if (isLeftPhase) {
      return rightFront > rightRear;
   }
   return leftFront > leftRear;
}

float BodyModel::getFootZMP(bool isLeft, const SensorValues &sensors) {
   if (walkCycle.isDoubleSupportPhase()) return 0;
   float ZMPF = 0;
   float totalPressure = 0;
   if (isLeft) {
      totalPressure += sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
                         + sensors.sensors[Sensors::LFoot_FSR_FrontRight]
                         + sensors.sensors[Sensors::LFoot_FSR_RearLeft]
                         + sensors.sensors[Sensors::LFoot_FSR_RearRight];
   } else {
      totalPressure += sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
                         + sensors.sensors[Sensors::RFoot_FSR_FrontRight]
                         + sensors.sensors[Sensors::RFoot_FSR_RearLeft]
                         + sensors.sensors[Sensors::RFoot_FSR_RearRight];
   }
   if (isLeft) {
      ZMPF =
            (75.0f) * sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
          + (-45.0f) * sensors.sensors[Sensors::LFoot_FSR_RearLeft]
          + (75.0f) * sensors.sensors[Sensors::LFoot_FSR_FrontRight]
          + (-45.0f) * sensors.sensors[Sensors::LFoot_FSR_RearRight];
   } else {
      ZMPF =
           (75.0f) * sensors.sensors[Sensors::RFoot_FSR_FrontRight]
         + (-45.0f) * sensors.sensors[Sensors::RFoot_FSR_RearRight]
         + (75.0f) * sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
         + (-45.0f) * sensors.sensors[Sensors::RFoot_FSR_RearLeft];
   }
   if (totalPressure == 0) return  0;
   ZMPF /= totalPressure;
   return ZMPF;
}

float BodyModel::getHorizontalFootZMP(bool isLeft, const SensorValues &sensors) {
   float ZMPF = 0;
   float totalPressure = 0;
   if (isLeft) {
      totalPressure += sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
                         + sensors.sensors[Sensors::LFoot_FSR_FrontRight]
                         + sensors.sensors[Sensors::LFoot_FSR_RearLeft]
                         + sensors.sensors[Sensors::LFoot_FSR_RearRight];
   } else {
      totalPressure += sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
                         + sensors.sensors[Sensors::RFoot_FSR_FrontRight]
                         + sensors.sensors[Sensors::RFoot_FSR_RearLeft]
                         + sensors.sensors[Sensors::RFoot_FSR_RearRight];
   }
   if (isLeft) {
      ZMPF =
            (20.0f) * sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
          + (20.0f) * sensors.sensors[Sensors::LFoot_FSR_RearLeft]
          + (-20.0f) * sensors.sensors[Sensors::LFoot_FSR_FrontRight]
          + (-20.0f) * sensors.sensors[Sensors::LFoot_FSR_RearRight];
   } else {
      ZMPF =
           (-20.0f) * sensors.sensors[Sensors::RFoot_FSR_FrontRight]
         + (-20.0f) * sensors.sensors[Sensors::RFoot_FSR_RearRight]
         + (20.0f) * sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
         + (20.0f) * sensors.sensors[Sensors::RFoot_FSR_RearLeft];
   }
   if (totalPressure == 0) return  0;
   ZMPF /= totalPressure;
   return ZMPF;
}
