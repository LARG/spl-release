#pragma once

#include <boost/shared_ptr.hpp>
#include <alsa/asoundlib.h>
#include <alsa/control.h>
// #include <alcommon/almodule.h>
// #include <alaudio/alsoundextractor.h>

class MemoryFrame;
class AudioProcessingBlock;

class AudioWrapper {
// class AudioWrapper : public AL::ALSoundExtractor {
  public:
    // AudioWrapper(boost::shared_ptr<AL::ALBroker> broker, const std::string & name);
    AudioWrapper(const std::string & name);

    virtual ~AudioWrapper();
    void init();
    void process(const int & nbOfChannels,
                 const int & nbrOfSamplesByChannel);
                 // const int & nbrOfSamplesByChannel,
                 // const AL_SOUND_FORMAT * buffer,
                 // const AL::ALValue & timeStamp);
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
};
