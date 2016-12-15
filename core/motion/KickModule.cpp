#include <motion/KickModule.h>

KickModule::KickModule() {
}

KickModule::~KickModule() {
}

void KickModule::specifyMemoryDependency() {
  requiresMemoryBlock("frame_info");
  requiresMemoryBlock("processed_joint_commands");
  requiresMemoryBlock("processed_joint_angles");
  requiresMemoryBlock("kick_module");
  requiresMemoryBlock("kick_params");
  requiresMemoryBlock("kick_request");
  requiresMemoryBlock("odometry");
  requiresMemoryBlock("robot_info");
  requiresMemoryBlock("processed_sensors");
  requiresMemoryBlock("speech");
  requiresMemoryBlock("walk_info");
  requiresMemoryBlock("walk_request");
}

void KickModule::specifyMemoryBlocks() {
  getMemoryBlock(frame_info_,"frame_info");
  getMemoryBlock(commands_,"processed_joint_commands");
  getMemoryBlock(joint_angles_,"processed_joint_angles");
  getOrAddMemoryBlock(kick_module_,"kick_module");
  getOrAddMemoryBlock(kick_params_,"kick_params");
  getMemoryBlock(kick_request_,"kick_request");
  getMemoryBlock(odometry_,"odometry");
  getMemoryBlock(robot_info_, "robot_info");
  getMemoryBlock(sensors_,"processed_sensors");
  getMemoryBlock(speech_, "speech");
  getMemoryBlock(walk_info_,"walk_info");
  getMemoryBlock(walk_request_,"walk_request");
}

void KickModule::initSpecificModule() {
}

void KickModule::processFrame() {
}

int KickModule::getFramesInState() {
  return 0;
}

float KickModule::getTimeInState() {
  return 0.0f;
}

float KickModule::getMillisecondsInState() {
  return 0.0f;
}

void KickModule::processKickRequest() {
}

void KickModule::startKick() {
}

void KickModule::setHead() {
}

void KickModule::initStiffness() {
}

void KickModule::setLegStiffness(float stiff) {
}

void KickModule::transitionToState(KickState::State state) {
}

bool KickModule::chooseKickLeg() {
  return true;
}

bool KickModule::checkKickValidity() {
  return true;
}

bool KickModule::handleAiming() {
  return true;
}

void KickModule::kick() {
}

void KickModule::calcSwingSplinePts() {
}

void KickModule::setSwingSpline(int num_pts,double timesInMs[], double xs[], double ys[], double zs[]) {
}

void KickModule::sendSplineCOMCommands(const Vector3<float> &com_in) {
}

void KickModule::getSwingTargets(Vector3<float> &align, Vector3<float> &kick) {
}


void KickModule::calcBallPosWRTSwingLeg() {
}

void KickModule::setKickOdometry() {
}

void KickModule::calcJointTargets(const Vector3<float> &com_target, const Pose3D &swing_rel_stance, bool is_left_swing, float command_angles[NUM_JOINTS], bool move_com, float roll) {
}

void KickModule::setArms(float command_angles[NUM_JOINTS]) {
}

void KickModule::calcCenterOfMass(float *command_angles, Vector3<float> &center_of_mass, bool stance_is_left, float tilt_roll_factor){
}

void KickModule::commandLegsRelativeToTorso(float *command_angles, Pose3D left_target, Pose3D right_target, float /*tilt*/, float roll, float /*tilt_roll_factor*/, bool left_compliant, bool right_compliant) {
}


void KickModule::adjustBalance(bool left_swing, float command_angles[NUM_JOINTS]) {
}
