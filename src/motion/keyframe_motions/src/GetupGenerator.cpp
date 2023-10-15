#include <fstream>
#include <limits>
#include <cctype>
#include <unistd.h>
#include <limits.h>
#include <sstream>
#include "keyframe_motions/GetupGenerator.hpp"
#include <boost/algorithm/string.hpp>
#include "keyframe_motions/MotionDefs.hpp"

using namespace std;

GetupGenerator::GetupGenerator(std::string falldirection,rclcpp::Node* motionNode) : fall_direction(falldirection), motionNode(motionNode) {
   max_iter = 0;
   fallen = false;
   num_times_fallen = 0;
   current_time = NOT_RUNNING;
};

GetupGenerator::~GetupGenerator() {
   cout << "GetupGenerator destroyed" << std::endl;
};

void GetupGenerator::setConfig(std::string curr_motion, std::string motion_speed, std::string direction){
  motion_comm = curr_motion;
  speed = motion_speed;
  fall_direction = direction;
  // RCLCPP_INFO(motionNode->get_logger(), "setting motion with speed: %s\n",speed.c_str());
  // RCLCPP_INFO(motionNode->get_logger(), "setting motion to: %s\n",motion_comm.c_str());

}

bool GetupGenerator::isActive() {
   return current_time != NOT_RUNNING;
};

void GetupGenerator::reset() {
   current_time = 0;
   joints.clear();
}

void GetupGenerator::stop() {
   current_time = NOT_RUNNING;
   joints.clear();
}

JointValues GetupGenerator::makeJoints(const SensorValues &sensors) {

   JointValues j;
   if (isActive()) {
     float ang[2] = {RAD2DEG(sensors.sensors[Sensors::InertialSensor_AngleX]),
                     RAD2DEG(sensors.sensors[Sensors::InertialSensor_AngleY])};

     JointValues newJoints = sensors.joints;
     for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
        newJoints.stiffnesses[i] = 1.0f;
     }

     if (current_time == 0){
        interpolate(newJoints,0,sensors);
        readOptions(sensors);
     }

     current_time++;

     if (current_time < joints.size()){
        j = joints[current_time];
      //   RCLCPP_INFO(motionNode->get_logger(), "joints.size() %d\n",joints.size());
      //   RCLCPP_INFO(motionNode->get_logger(), "current_time %d\n",current_time);
     } else{
       //this means we have successfully completed the getup
       j = joints[joints.size()-1];
       current_time = NOT_RUNNING;
     }
   } else{
     j = joints[joints.size()-1];
   }
   return j;
};

void GetupGenerator::interpolate(JointValues newJoint, int duration, const SensorValues &sensors) {
   // RCLCPP_INFO(motionNode->get_logger(), "Current duration %d\n",duration);
   // RCLCPP_INFO(motionNode->get_logger(), "Joints.empty() %d\n",joints.empty());

   if (joints.empty()) {
      max_iter = duration / (1000.0 * MOTION_DT);
      // Reserve space for the interpolation when the generator
      // first called
      // newJoint.angles[Joints::LShoulderPitch] = 1.570796;
      // newJoint.angles[Joints::RShoulderPitch] = 1.570796;
      for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
        newJoint.angles[i] = sensors.joints.angles[i];
     }
      for (int i = 0; i < max_iter; i++) {
         joints.push_back(newJoint);
      }
      joints.push_back(newJoint);
   } else {
      int inTime = 0;
      float offset[Joints::NUMBER_OF_JOINTS];

      if (duration != 0) {
         inTime = duration / (1000.0 * MOTION_DT);
         JointValues currentJoint = joints.back();

         // RCLCPP_INFO(motionNode->get_logger(), "Current LShoulderPitch %f\n",currentJoint.angles[Joints::LShoulderPitch]);
         // RCLCPP_INFO(motionNode->get_logger(), "New LShoulderPitch %f\n",newJoint.angles[Joints::LShoulderPitch]);

         // Calculate the difference between new joint and the previous joint
         for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
            offset[i] = (newJoint.angles[i] - currentJoint.angles[i]) / inTime;
         }

         for (int i = 0; i < inTime; i++) {
            JointValues inJoint;
            for (int j = 0; j < Joints::NUMBER_OF_JOINTS; j++) {
               inJoint.angles[j] = joints.back().angles[j] + offset[j];
               inJoint.stiffnesses[j] = newJoint.stiffnesses[j];
            }
            joints.push_back(inJoint);
         }
      } else {

         JointValues firstJoint = joints.at(max_iter);
         // Calculate the difference between the joint at MAX_ITER position
         // with the new joint
         for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
            offset[i] = (firstJoint.angles[i] - newJoint.angles[i]) / max_iter;
         }

         joints[0] = newJoint;
         for (int i = 1; i < max_iter; i++) {
            for (int j = 0; j < Joints::NUMBER_OF_JOINTS; j++) {
               joints[i].angles[j] = joints[i - 1].angles[j] + offset[j];
               joints[i].stiffnesses[j] = firstJoint.stiffnesses[j];
            }
         }
      }
   }
}

bool GetupGenerator::individualPathExists(std::string individualPath, std::string bodyName) {
   ifstream ifs;
   ifs.open(string(individualPath + "/" + bodyName + "_" + file_name + ".pos").c_str());

   if (!ifs) {
      return false;
   } else {
      cout << "GetupGenerator: Individual file path found." << endl;
      ifs.close();
      return true;
   }
}


void GetupGenerator::constructPose(const SensorValues &sensors)
{

    // RCLCPP_INFO(motionNode->get_logger(), "constructPose() is called\n");

    /*
    * Check for individual pos files, if they exist.
    * Getups require the entire body of the nao, so individual pos fles exist
    * to account for multiple joint offsets. If no file exists for them,
    * then simply use the default pos file.
    */

   //  bool useIndividalPath = individualPathExists(individualPath, bodyName);
   bool useIndividalPath = false;

    ifstream in;
    if (useIndividalPath)
    {
        in.open(string(individualPath + "/" + bodyName + "_" + file_name + ".pos").c_str());
    }
    else
    {
        in.open(string(path + "/" + file_name + ".pos").c_str());
    }

    cout << "GetupGenerator(" << file_name << ") creating" << endl;

    // RCLCPP_INFO(motionNode->get_logger(), "Read file %s\n" , string(path + "/" + file_name + ".pos").c_str());
    if (!in.is_open())
    {
        cout << "GetupGenerator cannot open " << file_name << endl;
        // RCLCPP_INFO(motionNode->get_logger(), "Cannot open file\n");
    }
    else
    {
        JointValues newJoint(0);

        // Set to default 1 stiffness
        for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
            newJoint.stiffnesses[i] = 1.0f;

        while (!in.eof())
        {

            std::string line;
            std::getline(in, line);

            // RCLCPP_INFO(motionNode->get_logger(), "Reading line %s\n", line.c_str());

            if (boost::starts_with(line, "!"))
            {
                boost::erase_all(line, "!");

                std::istringstream ss(line);

                float angle;
                for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
                {
                    ss >> angle;
                    newJoint.angles[i] = DEG2RAD(angle);
                }

                int duration;
                ss >> duration;

                interpolate(newJoint, duration, sensors);

                // Set to default 1 stiffness
                for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
                    newJoint.stiffnesses[i] = 1.0f;
            }

            if (boost::starts_with(line, "$"))
            {
                boost::erase_all(line, "$");

                std::istringstream ss(line);

                float stiffness;
                for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
                {
                    ss >> stiffness;
                    newJoint.stiffnesses[i] = stiffness;
                }

            }
        }
        in.close();
    }
    cout << "GetupGenerator(" << file_name << ") created" << endl;
}

void GetupGenerator::chooseGetup(){
   // file_name = "getupBack";

   if (motion_comm=="getup" && fall_direction == "BACK"){
       if (speed == "FAST"){
          file_name = "getupBack";
       } else if (speed == "MODERATE"){
          file_name = "getupBack";
       } else if (speed == "SLOW"){
          file_name = "getupBackSlow";
       }

   } else if (motion_comm=="getup" && fall_direction == "FRONT"){
       if (speed == "FAST"){
          file_name = "getupFrontFast"; // we don't want goalie to do a fast getup ... or do we?
       } else if (speed == "MODERATE"){
          file_name = "getupFront";
       } else if (speed == "SLOW"){
          file_name = "getupFrontSlow";
       }

   } else if (motion_comm == "stand"){

      file_name = "stand";

   }else if (motion_comm == "goaliesit"){

      file_name = "goalieSit";

   }else if (motion_comm == "sit"){

      file_name = "sit";

   } else if (motion_comm == "unstiff"){
     file_name = "unstiff";
   }

   else {
      RCLCPP_INFO(motionNode->get_logger(), "Cant read motion: %s\n", motion_comm.c_str());
   }

   // RCLCPP_INFO(motionNode->get_logger(), "Reading speed: %s\n", speed.c_str());
   // RCLCPP_INFO(motionNode->get_logger(), "Reading dir: %s\n", fall_direction.c_str());


}

void GetupGenerator::readOptions(const SensorValues &sensors) {
   chooseGetup();
   constructPose(sensors);
}
