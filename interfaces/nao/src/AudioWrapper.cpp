#include "AudioWrapper.h"
#include "naointerface.h"
#include <memory/AudioProcessingBlock.h>
// #include <alcommon/alproxy.h>

// AudioWrapper::AudioWrapper(boost::shared_ptr<AL::ALBroker> broker, const std::string & name) :
//   AL::ALSoundExtractor(broker, name), memory_(naointerface::MEMORY_INSTANCE) {
// }

AudioWrapper::AudioWrapper(const std::string & name) {}

AudioWrapper::~AudioWrapper() {
  // stopDetection();
}

void AudioWrapper::init() {
    snd_pcm_stream_t stream = SND_PCM_STREAM_CAPTURE;
    /* Init pcm_name. Later we */
    /* will make this configurable ;-)     */
    pcm_name = strdup("plughw:0,0");

    /* Allocate the snd_pcm_hw_params_t structure on the stack. */
    snd_pcm_hw_params_alloca(&hwparams);
    
    if (snd_pcm_open(&pcm_handle, pcm_name, stream, 0) < 0) {
      fprintf(stderr, "Error opening PCM device %s\n", pcm_name);
      exit(1);
    }

    spectra_per_sec_ = 47;
    window_size_ = 1024;
    int rate = spectra_per_sec_ * window_size_; /* Sample rate */
    int exact_rate;   /* Sample rate returned by */
                      /* snd_pcm_hw_params_set_rate_near */ 
    int dir;          /* exact_rate == rate --> dir = 0 */
                      /* exact_rate < rate  --> dir = -1 */
                      /* exact_rate > rate  --> dir = 1 */
    int periods = 2;       /* Number of periods */
    snd_pcm_uframes_t periodsize = 8192; /* Periodsize (bytes) */

    /* Fill it in with default values. */
    snd_pcm_hw_params_any(pcm_handle, hwparams);

    /* Set the desired hardware parameters. */

    /* Signed 16-bit little-endian format */
    snd_pcm_hw_params_set_format(pcm_handle, hwparams,
                                 SND_PCM_FORMAT_S16_LE);
    
    /* One channel*/
    snd_pcm_hw_params_set_channels(pcm_handle, hwparams, AudioProcessingBlock::NumChannels);

    /* bits/second sampling rate */
    unsigned int val = AudioProcessingBlock::SampleRate;
    // spectra_per_sec_ * window_size_;
    snd_pcm_hw_params_set_rate_near(pcm_handle, hwparams,
                                    &val, &dir);
    
    /* Set period size to 32 frames. */
    frames = window_size_;
    snd_pcm_hw_params_set_period_size_near(pcm_handle,
                                hwparams, &frames, &dir);
    /* Write the parameters to the driver */
    int rc = snd_pcm_hw_params(pcm_handle, hwparams);
    if (rc < 0) {
        fprintf(stderr,
              "unable to set hw parameters: %s\n",
              snd_strerror(rc));
        exit(1);
    }

    /* Use a buffer large enough to hold one period */
    snd_pcm_hw_params_get_period_size(hwparams,
                                        &frames, &dir);
    int size = frames * 2 * AudioProcessingBlock::NumChannels; /* 2 bytes/sample, 4 channels */
    buffer_ = (char *) malloc(size);


 //    //using all 4 channels at 48000 Hz, default mode
 //    audioDevice->callVoid("setClientPreferences",
 //        getName(),                //Name of this module
 //        AudioProcessingBlock::SampleRate,  //48000 Hz requested
 //        (int)AL::ALLCHANNELS,        //All Channels requested
 //        1                         //Deinterleaving is needed here
 //    );
 //  }
 //  else {
 //    ////using single channel at 16000 Hz
 //    audioDevice->callVoid("setClientPreferences",
 //        getName(),                //Name of this module
 //        AudioProcessingBlock::SampleRate,  //16000 Hz requested
 //        (int)AL::FRONTCHANNEL,        //Front Channels requested
 //        0                        //Deinterleaving is not needed here
 //        );
 //  }
	// if(memory_)
	// 	memory_->getOrAddBlockByName(audio_block_,"audio_processing", MemoryOwner::VISION);
  // startDetection();
}

// void AudioWrapper::process(const int & nbOfChannels, const int & nbrOfSamplesByChannel, const AL_SOUND_FORMAT * buffer, const AL::ALValue & timestamp) {
void AudioWrapper::process(const int & nbOfChannels, const int & nbrOfSamplesByChannel) {
  int rc = snd_pcm_readi(pcm_handle, buffer_, frames);
  if (rc == -EPIPE) {
    printf("Overrun occured\n");
    return;
  }
  else if (rc < 0) {
    fprintf(stderr,
            "error from read: %s\n",
            snd_strerror(rc));
  }
  else if (rc != (int)frames) {
    fprintf(stderr, "short read, read %d frames\n", rc);
  }
  
  // int ts1 = timestamp[0], ts2 = timestamp[1];
  // unsigned long long ts = ts1;
  // ts *= 1000000ul;
  // ts += ts2;
  // audio_block_->timestamp_ = ts;
  
  /* Each sample is 16 bits or 2 bytes */
  memcpy(audio_block_->buffer_.data(), buffer_, 2 * AudioProcessingBlock::BufferSampleCount);
  
  /* We sample the audio here and write it to the memory block */
}
