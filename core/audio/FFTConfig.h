#pragma once

#include <common/YamlConfig.h>
#include <memory/AudioProcessingBlock.h>

class FFTConfig : public YamlConfig {
  public:
    FFTConfig();

    int channel_offset;
    int window_size; // The number of samples for each FFT window
    int window_step; // The number of samples to skip between each FFT window
    int maximum_samples; // A hard maximum number of samples per FFT run to reduce computation time
    inline float window_ms(int count = 1) const { 
      return count * static_cast<float>(window_size) * AudioProcessingBlock::MsPerSample;
    }
    inline int window_count(float ms) const {
      return std::floor(ms * AudioProcessingBlock::SamplesPerMs / window_step);
    }
    inline int samples_per_spectrum() const { 
      return std::min<int>(AudioProcessingBlock::SamplesPerChannelPerBuffer, maximum_samples);
    }
    inline int spectrum_count() const { return (samples_per_spectrum() - window_size) / window_step + 1; }
    inline int spectrum_length() const { return window_size_half(); }
    inline int window_size_half() const { return window_size / 2 + 1; }
    inline int frequency_index(float frequency) const { 
      return frequency * static_cast<float>(window_size) / AudioProcessingBlock::SampleRate;
    }
    inline float index_frequency(int index) const {
      return index * static_cast<float>(AudioProcessingBlock::SampleRate) / window_size;
    }
    inline bool operator==(const FFTConfig& other) {
      return 
        channel_offset == other.channel_offset && 
        window_size == other.window_size &&
        window_step == other.window_step &&
        maximum_samples == other.maximum_samples
      ;
    }

  private:
    void deserialize(const YAML::Node& node) override;
    void serialize(YAML::Emitter& emitter) const override;
};
