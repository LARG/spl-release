#include <audio/AudioModule.h>

#include <memory/FrameInfoBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/AudioProcessingBlock.h>
#include <memory/TeamPacketsBlock.h>

#include <audio/KangarooDetector.h>
#include <audio/KeypointDetector.h>
#include <audio/Logging.h>
#include <common/WorldObject.h>
#include <common/Util.h>
#include <VisionCore.h>

AudioModule::AudioModule() : tlogger_(textlogger) {
}

AudioModule::~AudioModule() {
}

void AudioModule::updateConfig(FFTConfig fftConfig) {
  if(fft_config_ == fftConfig) return;
  fft_config_ = fftConfig;
  detector_->updateConfig();
}

const FFT& AudioModule::fft() const { 
  return static_cast<const FFT&>(detector_->fft());
}

void AudioModule::specifyMemoryDependency() {
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("audio_processing");
  requiresMemoryBlock("team_packets");
}

void AudioModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.audio_processing,"audio_processing");
  getOrAddMemoryBlock(cache_.team_packets,"team_packets");
}

void AudioModule::initSpecificModule() {
  bool loaded = fft_config_.load(util::format("%s/whistle_fft_config.yaml", util::cfgpath(util::Models)));
  if(!loaded) {
    throw std::runtime_error("Unable to load whistle fft config.");
  }
  printf("Loaded Whistle FFT, Size: %i, Step: %i, Max Samples: %i\n", fft_config_.window_size, fft_config_.window_step, fft_config_.maximum_samples);
  detector_ = std::make_unique<KeypointDetector>(cache_, tlogger_, fft_config_);
  whistle_heard_frame_ = 0;
}

void AudioModule::initFromMemory() {
  whistle_heard_frame_ = cache_.audio_processing->whistle_heard_frame_ = 0;
}

void AudioModule::processCommunications() {
  auto filter = std::make_unique<WhistleUKF>(*detector_->filter());
  WhistleObservation obs;
  for(int i = WO_TEAM_FIRST; i <= WO_TEAM_LAST; i++) {
    if(i == cache_.robot_state->WO_SELF) continue;
    const auto& relay = cache_.team_packets->relayData[i];
    uint32_t heard_frame = static_cast<uint32_t>(relay.bvrData.whistleHeardFrame);
    float score = static_cast<float>(relay.bvrData.whistleScore);
    float sd = static_cast<float>(relay.bvrData.whistleSd);
    // If the whistle score is too stale then just use default values
    if (heard_frame - cache_.frame_info->frame_id > 20) {
        score = 0.;
        sd = 1.;
    }
    score = WhistleUKF::sanitizeState(score);
    sd = WhistleUKF::sanitizeSd(sd);
    tlog(65, "Processing score %2.2f, sd %2.2f from teammate %i", score, sd, i);
    obs.teammateScores.push_back(score);
    obs.teammateSds.push_back(sd);
  }
  tlog(65, "Processing team whistles: %s", obs);
  tlog(65, "Filter (pre-update): %s", *filter);
  filter->update(obs);
  tlog(65, "Filter (post-update): %s", *filter);
  if(filter->score() > detector_->threshold()) {
    tlog(65, "Filter score of %2.2f exceeds threshold %2.2f, setting whistle heard to %i", filter->score(), detector_->threshold(), cache_.frame_info->frame_id);
    cache_.audio_processing->teammate_heard_frame_ = cache_.frame_info->frame_id;
  }
}

void AudioModule::processFrame() {
  processCommunications();
  if(!VisionCore::inst_->rconfig().audio_enabled) return;
  bool success = false;
  bool isOff = cache_.audio_processing->state_ == AudioProcessingBlock::Off;
  bool hasDetected = detector_->detected();
  bool hasHadTimeToBroadcast = cache_.frame_info->frame_id - whistle_heard_frame_ > 30;
  if(isOff && hasDetected && hasHadTimeToBroadcast) {
    detector_->reset();
    scores_ = { 0.0f };
  }
  if(timestamp_ != cache_.audio_processing->timestamp_ && cache_.audio_processing->state_ == AudioProcessingBlock::Detecting) {
    timestamp_ = cache_.audio_processing->timestamp_;
    AudioTimer::Start("audio detect");
    success = detector_->detect();
    //TODO: adjust the UKF instead of using a max queue
    scores_.push_back(detector_->filter()->score());
    if(scores_.size() > 10) scores_.pop_front();
    AudioTimer::Stop("audio detect");
    tlog(40, "frame %i has new timestamp %llu, detect: %s", cache_.frame_info->frame_id, timestamp_,
      success ? "SUCCESS" : "FAILURE"
    );
  }
  if(success) {
    printf("Whistle HEARD\n\n\n");
    whistle_heard_frame_ = cache_.frame_info->frame_id;
  }
  cache_.audio_processing->whistle_heard_frame_ = whistle_heard_frame_;
  cache_.audio_processing->whistle_score_ = *std::max_element(scores_.begin(), scores_.end());
  cache_.audio_processing->whistle_sd_ = detector_->filter()->sd();
}

AudioDetector* AudioModule::detector() { return detector_.get(); }
