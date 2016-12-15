#include <audio/AudioModule.h>

#include <memory/FrameInfoBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/AudioProcessingBlock.h>
#include <memory/TeamPacketsBlock.h>

AudioModule::AudioModule() : tlogger_(textlogger) {
}

AudioModule::~AudioModule() {
}

void AudioModule::specifyMemoryDependency() {
  requiresMemoryBlock("audio_processing");
}

void AudioModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.audio_processing,"audio_processing");
}

void AudioModule::initSpecificModule() {
  whistle_heard_frame_ = 0;
}

void AudioModule::initFromMemory() {
  whistle_heard_frame_ = cache_.audio_processing->whistle_heard_frame_ = 0;
}

void AudioModule::processCommunications() {
}

void AudioModule::processFrame() {
}
