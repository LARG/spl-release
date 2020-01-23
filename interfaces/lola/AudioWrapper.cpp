#include "AudioWrapper.h"
#include "naointerface.h"
#include <memory/AudioProcessingBlock.h>
#include <iostream>
#include <stdlib.h>
// #include <alcommon/alproxy.h>

// AudioWrapper::AudioWrapper(boost::shared_ptr<AL::ALBroker> broker, const std::string & name) :
//   AL::ALSoundExtractor(broker, name), memory_(naointerface::MEMORY_INSTANCE) {
// }

AudioWrapper::AudioWrapper() {}

AudioWrapper::~AudioWrapper() {
  snd_pcm_close(pcm_handle);
  // stopDetection();
}

void AudioWrapper::init(AudioProcessingBlock *block) {
    ts = 0;
    audio_block_ = block;
    // printf("In AW init\n"); 
    snd_pcm_stream_t stream = SND_PCM_STREAM_CAPTURE;
    /* Init pcm_name. Later we */
    /* will make this configurable ;-)     */
    pcm_name = strdup("plughw:0,0");
    // printf("create name\n");

    if (snd_pcm_open(&pcm_handle, pcm_name, stream, SND_PCM_NONBLOCK) < 0) {
      printf("Error opening PCM device %s\n", pcm_name);
      exit(1);
    }

    /* Allocate the snd_pcm_hw_params_t structure on the stack. */
    snd_pcm_hw_params_alloca(&hwparams);
    // printf("Allocated hw_params\n");
    

    // spectra_per_sec_ = 48;
    window_size_ = 1200;
    // int rate = spectra_per_sec_ * window_size_; /* Sample rate */
    int exact_rate;   /* Sample rate returned by */
                      /* snd_pcm_hw_params_set_rate_near */ 
    int dir;          /* exact_rate == rate --> dir = 0 */
                      /* exact_rate < rate  --> dir = -1 */
                      /* exact_rate > rate  --> dir = 1 */
    int periods = 2;       /* Number of periods */
    // snd_pcm_uframes_t periodsize = 8192; /* Periodsize (bytes) */

    // printf("Initialized some variables\n");

    /* Fill it in with default values. */
    snd_pcm_hw_params_any(pcm_handle, hwparams);

    // printf("Set params to default values\n");
    /* Set the desired hardware parameters. */

    /*
       Interleaved mode. Interleaves the values of multiple channels
       If contiguous blocks from a channel are needed, set this to SND_PCM_ACCESS_RW_NONINTERLEAVED
    */
    snd_pcm_hw_params_set_access(pcm_handle, hwparams, SND_PCM_ACCESS_RW_INTERLEAVED);

    /* Signed 16-bit little-endian format */
    snd_pcm_hw_params_set_format(pcm_handle, hwparams,
                                 SND_PCM_FORMAT_S16_LE);
    
    // printf("Set output to 16 bit format\n");
    /* 4 channels*/
    snd_pcm_hw_params_set_channels(pcm_handle, hwparams, AudioProcessingBlock::NumChannels);
    //AudioProcessingBlock::NumChannels);
    // printf("Specified reading 4 channels\n");

    /* bits/second sampling rate */
    unsigned int val = AudioProcessingBlock::SampleRate;
    std::cout << val << std::endl;
    // spectra_per_sec_ * window_size_;
    snd_pcm_hw_params_set_rate_near(pcm_handle, hwparams,
                                    &val, NULL);  // &dir);
    // printf("Set sampling rate\n");
    // std::cout << val << std::endl;

    /* Set period size to 600 frames. (48000 frames per second at 80 Hz) */
    frames = window_size_;
    snd_pcm_hw_params_set_period_size_near(pcm_handle,
                               hwparams, &frames, &dir);
    /* Write the parameters to the driver */
    int rc = snd_pcm_hw_params(pcm_handle, hwparams);
    // printf("Wrote params to driver\n");
    if (rc < 0) {
        fprintf(stderr,
              "unable to set hw parameters: %s\n",
              snd_strerror(rc));
        exit(1);
    }

    /* Use a buffer large enough to hold one period */
    snd_pcm_hw_params_get_period_size(hwparams,
                                        &frames, &dir);
    // printf("Get period size\n");
    // std::cout << frames << std::endl;
    
    int size = 2 * AudioProcessingBlock::NumChannels; /* 2 bytes/sample, 4 channels */
    buffer_ = (char*) calloc(frames, size);
    // printf("Create buffer\n");

    // printf("arguments to read: ");
    // std::cout << pcm_handle << std::endl;
    // rc = snd_pcm_readi(pcm_handle, buffer_, frames);
    // printf("Read status: %d\n", rc);
    // if (rc < 0) {
    //   fprintf(stderr,
    //           "error from read: %s\n",
    //           snd_strerror(rc));
    // }
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
    //   memory_->getOrAddBlockByName(audio_block_,"audio_processing", MemoryOwner::VISION);
  // startDetection();
}

void AudioWrapper::process() {
  ts += 1;
  /*
    Read the audio every 2 iterations
    This is because we used to process 1024 samples
  */
  if(ts % 2 == 0)
    return;
  // std::cout << "Attempting to read audio" << std::endl;
  int rc = snd_pcm_readi(pcm_handle, (void *)buffer_, frames);
  // std::cout << rc << std::endl;
  if (rc == -EPIPE) {
    printf("Overrun occured\n");
    return;
  }
  else if (rc == -EBADFD) {
    printf("PCM is not in right state\n");
  }
  else if (rc == -ESTRPIPE) {
    printf("Suspend event occurred\n");
  }
  else if (rc < 0) {
    fprintf(stderr,
            "error from read: %s\n",
            snd_strerror(rc));
  }
  else if (rc != (int)frames) {
    // fprintf(stderr, "short read, read %d frames\n", rc);
    memcpy(audio_block_->buffer_.data(), buffer_, 2 * AudioProcessingBlock::BufferSampleCount);
    audio_block_->num_samples = rc;
    audio_block_->timestamp_ = ts;
    // std::cout << "Timestamp: " << ts << std::endl;
  }
  else {
    // printf("Read audio\n");
    /* Each sample is 16 bits or 2 bytes */
    memcpy(audio_block_->buffer_.data(), buffer_, 2 * AudioProcessingBlock::BufferSampleCount);
    audio_block_->num_samples = rc;
    audio_block_->timestamp_ = ts;
    // std::cout << "Timestamp: " << ts << std::endl;
    /* We sample the audio here and write it to the memory block */
  }
  
  // int ts1 = timestamp[0], ts2 = timestamp[1];
  // unsigned long long ts = ts1;
  // ts *= 1000000ul;
  // ts += ts2;
  // audio_block_->timestamp_ = ts;
  
  
}
