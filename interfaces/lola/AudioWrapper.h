#pragma once

#include <boost/shared_ptr.hpp>
#include <alsa/asoundlib.h>
#include <alsa/control.h>
#include <ctime>
// #include <alcommon/almodule.h>
// #include <alaudio/alsoundextractor.h>

class MemoryFrame;
class AudioProcessingBlock;

class AudioWrapper {
// class AudioWrapper : public AL::ALSoundExtractor {
  public:
    // AudioWrapper(boost::shared_ptr<AL::ALBroker> broker, const std::string & name);
    AudioWrapper();

    virtual ~AudioWrapper();
    void init(AudioProcessingBlock*);
    void process();
  private:
    AudioProcessingBlock *audio_block_;
    MemoryFrame* memory_;
    snd_pcm_t *pcm_handle;
    snd_pcm_hw_params_t *hwparams;
    char *pcm_name;
    char *buffer_;
    snd_pcm_uframes_t frames;
    int spectra_per_sec_;
    int window_size_;
    // time_t timestamp;
    unsigned long long ts;
};
