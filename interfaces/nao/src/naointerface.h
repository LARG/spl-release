/**
 * @author Michael Quinlan
 *
 * This file was generated by Aldebaran Robotics ModuleGenerator
 */

#ifndef NAOINTERFACE_NAOINTERFACE_H
#define NAOINTERFACE_NAOINTERFACE_H

#include <boost/shared_ptr.hpp>
// #include <alcommon/almodule.h>
// #include <alaudio/alsoundextractor.h>
#include <string>

#include <memory/MemoryFrame.h>
#include <memory/Lock.h>

#include <memory/FrameInfoBlock.h>
#include <memory/SensorBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/LEDBlock.h>
#include <memory/WalkInfoBlock.h>
#include <memory/ALWalkParamBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/SpeechBlock.h>
#include <memory/RobotStateBlock.h>

#include "dcmwrapper.h"
#include "almotionwrapper.h"

/*
namespace AL
{
  class ALBroker;
  class ALMemoryProxy;
  class ALMotionProxy;
  class ALMemoryFastAccess;
  class ALTextToSpeechProxy;
  class DCMProxy;
  class ALSonarProxy;
}
*/
/**
 * DESCRIBE YOUR CLASS HERE
 */
class naointerface // : public AL::ALModule
{
  public:
    /**
     * Default Constructor.
     */
     // naointerface(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
     naointerface(const std::string& name);

    /**
     * Destructor.
     */
    virtual ~naointerface();
    
    void start();
    void stop();
    
    double getSystemTime();

    void postProcess(); // called immediately after sensors arrives
    void preProcess(); // called before sending joint commands

    void initMemory();
    void initFastAccess();

    void populateSensors();

    void preProcessJoints();
    void postProcessJoints();
    
    void initSonar();
    
    // some functions for audio capture
    void startAudioCapture();
    void stopAudioCapture();
    

    // Used for fast memory access
    // boost::shared_ptr<AL::ALMemoryFastAccess> fast_sensor_access_;
    // boost::shared_ptr<AL::ALMemoryProxy> al_memory_;
    // boost::shared_ptr<AL::ALMotionProxy> al_motion_;
    std::vector<std::string> sensor_keys_;
    std::vector<float> sensor_values_;

    // Class that handles most of the DCM calls
    DCMWrapper* dcmWrap_;
    ALMotionWrapper* al_motion_wrap_;

    // boost::shared_ptr<AL::ALSonarProxy> sonar_proxy_;
    // boost::shared_ptr<AL::ALTextToSpeechProxy> tts_proxy_;
    
    //sound detected
    //boost::shared_ptr<ALSoundBasedReaction> sound_detector_;
	//ALSoundBasedReaction *r;


    void initPositions();
    void setAllPositions(float positions[NUM_JOINTS]);
   
    // Used for postprocess sync with the DCM
    // ProcessSignalConnection dcm_postprocess_connection_;
    // ProcessSignalConnection dcm_preprocess_connection_;

    MemoryFrame *memory_;
    Lock *motion_lock_;

    FrameInfoBlock *frame_info_;
    SensorBlock* raw_sensors_;
    JointBlock* raw_joint_angles_;
    JointCommandBlock* raw_joint_commands_;
    JointBlock* processed_joint_angles_;
    JointCommandBlock* processed_joint_commands_;
    LEDBlock* led_commands_;
    WalkInfoBlock* walk_info_;
    ALWalkParamBlock* al_walk_param_;
    WalkRequestBlock* walk_request_;
    SpeechBlock *speech_;
    RobotStateBlock *robot_state_;

    static MemoryFrame* MEMORY_INSTANCE;

};

#endif  // NAOINTERFACE_NAOINTERFACE_H

