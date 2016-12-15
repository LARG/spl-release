#pragma once

#include <array>
#include <assert.h>
#include <iostream>
#include <algorithm>
#include <random>
#include <bitset>
#include <math/Common.h>
#include <common/Iterator.h>

class TextLogger;

struct Band {
  constexpr Band() : row(0), col(0), id(0), x(0), y(0), width(0), height(0) { }
  constexpr Band(int row, int col, int id, int x, int y, int width, int height) : row(row), col(col), id(id), x(x), y(y), width(width), height(height) { }
  int row, col, id;
  int x, y, width, height;
  int area() const { return width * height; }
  int centerX() const { return x + width / 2; }
  int centerY() const { return y + height / 2; }
  friend std::ostream& operator<<(std::ostream& os, const Band& band);
};

class WeightVector : public std::vector<int> {
  public:
    using Vec = std::vector<int>;
    WeightVector(std::size_t rows, std::size_t cols) : Vec(rows * cols, 0), rows_(rows), cols_(cols) { }
    int& operator()(int row, int col) {
      int id = row * cols_ + col;
      assert(id < size());
      return Vec::operator[](id);
    }
    int operator()(int row, int col) const {
      int id = row * cols_ + col;
      assert(id < size());
      return Vec::operator[](id);
    }
    using Vec::operator[];
    int& operator[](const Band& band) {
      assert(band.id < size());
      return Vec::operator[](band.id);
    }
    int operator[](const Band& band) const {
      assert(band.id < size());
      return Vec::operator[](band.id);
    }
  private:
    int rows_, cols_;
};

class BandSampler {
  public:
    // Each band is a window of the overall image. windowX() and windowY() are the window dimensions.
    inline int windowX() const { return winX_; }
    inline int windowY() const { return winY_; }

    // The area of the window
    inline int area() const { return winX_ * winY_; }

    // The bands view a subsampled portion of the image. resX() and resY() are the number of base
    // image columns/rows skipped during subsampling.
    inline int resX() const { return resX_; }
    inline int resY() const { return resY_; }

    // Step variables indicate the number of raw pixels skipped from one band row/column to the
    // next.
    inline int stepX() const { return stepX_; }
    inline int stepY() const { return stepY_; }

    // The number of rows/cols of bands.
    inline int rows() const { return rows_; }
    inline int cols() const { return cols_; }

    // The number of bands
    inline std::size_t size() const { return rows_ * cols_; }

    // When an object is detected, its band's weight is incremented to increase the
    // likelihood of selecting that band in the next frame. Nearby bands' weights are
    // incremented as well, with the amount decreasing as we get farther from the detection
    // band. Spread variables determine the rate of decrease, i.e. how far the weight increments
    // "spread" to other bands.
    inline int spreadX() const { return spreadX_; }
    inline int spreadY() const { return spreadY_; }
    
    // When integrating multiple samplers it is useful to weight them with an alpha value in [0,1]
    inline float& alpha() { return alpha_; }
    inline float alpha() const { return alpha_; }

    // Aliases
    using SampledBands = std::vector<Band>;
    using SampledIds = std::vector<int>;
    using Weights = WeightVector;

    class Iterator : public ItemIterator<BandSampler,int,Band> {
      public:
        using ItemIterator<BandSampler,int,Band>::ItemIterator;
        using ItemIterator<BandSampler,int,Band>::operator=;
        Band operator*() const final {
          return _container.band(_key);
        }
        using ItemIterator<BandSampler,int,Band>::operator++;
        using ItemIterator<BandSampler,int,Band>::operator--;
        Iterator operator--(int) {
          auto it = *this;
          _key--;
          return it;
        }
        Iterator operator++(int) {
          auto it = *this;
          _key++;
          return it;
        }
    };
    
    class ConstIterator : public ConstItemIterator<BandSampler,int,Band> {
      public:
        using ConstItemIterator<BandSampler,int,Band>::ConstItemIterator;
        Band operator*() const final {
          return _container.band(_key);
        }
        using ConstItemIterator<BandSampler,int,Band>::operator++;
        using ConstItemIterator<BandSampler,int,Band>::operator--;
        ConstIterator operator--(int) {
          auto it = *this;
          _key--;
          return it;
        }
        ConstIterator operator++(int) {
          auto it = *this;
          _key++;
          return it;
        }
    };

  // Iterator implementation
  public:
#ifndef SWIG
    inline auto begin() { return Iterator(*this, 0); }
    inline auto begin() const { return ConstIterator(*this, 0); }
    inline auto end() { return Iterator(*this, size()); }
    inline auto end() const { return ConstIterator(*this, size()); }
    inline auto cbegin() const { return ConstIterator(*this, 0); }
    inline auto cend() const { return ConstIterator(*this, size()); }
#endif

  public:
    BandSampler(int imageRows, int imageCols, int windowRows, int windowCols);
    inline void init(TextLogger* tl) { textlogger = tl; }
    void setPrior(const BandSampler& prior);
    void registerPixel(int y, int x);
    void registerBand(int row, int col);
    inline void registerBand(int bandId) { registerBand(bandId % cols(), bandId / cols()); }
    inline void registerBand(const Band& band) { registerBand(band.row, band.col); }
    inline void decayWeights() {
      std::for_each(weights_.begin(), weights_.end(), [](auto& w) { w *= 0.9f; });
    }
    SampledBands sample(int count);
    friend std::ostream& operator<<(std::ostream& os, const BandSampler& sampler);
    inline const Weights& weights() const { return weights_; }

  protected:
    void sampleUniform(const std::vector<bool>& used, SampledIds& ids);
    int binsearch(const Weights& weights, int target);
    inline int startX(int bandId) const {
      return (bandId % cols()) * stepX();
    }
    inline int startY(int bandId) const {
      return (bandId / cols()) * stepY();
    }
    inline int bandId(int row, int col) const {
      return row * cols() + col;
    }
    inline Band band(int bandId) const {
      auto col = bandId % cols();
      auto row = bandId / cols();
      return band(row, col);
    }
    inline Band band(int row, int col) const {
      auto bandId = this->bandId(row, col);
      return Band(row, col, bandId, startX(bandId), startY(bandId), windowX(), windowY());
    }
    inline int xToCol(int x) const {
      return std::min(x / stepX(), cols() - 1);
    }
    inline int yToRow(int y) const {
      return std::min(y / stepY(), rows() - 1);
    }
    std::vector<Band> areaBands(int y, int x) {
      std::vector<Band> bands;
      int startY = y - windowY() + 1;
      startY += static_math::posmod(-startY, stepY());
      int startX = x - windowX() + 1;
      startX += static_math::posmod(-startX, stepX());

      for(int j = startY; j <= y; j += stepY()) {
        for(int i = startX; i <= x; i += stepX()) {
          int row = yToRow(j), col = xToCol(i);
          bands.push_back(band(row,col));
        }
      }
      return bands;
    }

  private:
    int 
      winX_, winY_,
      stepX_, stepY_,
      resX_, resY_,
      rows_, cols_, 
      spreadX_, spreadY_,
      imageRows_, imageCols_
    ;
    Weights weights_;
    float alpha_;
    TextLogger* textlogger = nullptr;
};
