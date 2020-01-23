#pragma once

#include <vector>
#include <stdint.h>
#include <audio/Types.h>

class FFT {
  public:
    virtual ~FFT();
    virtual Spectra computeSpectra(const int16_t* data, int length, short channels) const = 0;
    virtual void processSamples(const int16_t* data, int length, short channels) const = 0;
};
