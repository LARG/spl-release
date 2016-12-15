#include <vision/BandSampler.h>
#include <common/Random.h>
#include <common/tinyformat.h>
#include <common/Profiling.h>
#include <vision/Logging.h>

#define SQ(x) ((x) * (x))
#define NEAREST_POWER_OF_2(x) (1 << static_cast<int>(log2(x) + 0.5f)) 

using namespace std;

BandSampler::BandSampler(int imageRows, int imageCols, int windowRows, int windowCols)
  : 
  winX_(windowCols), winY_(windowRows), 
  stepX_(10),
  stepY_(10),
  resX_(1), resY_(winY_ / 8),
  rows_((imageRows - winY_ + 1) / stepY_),
  cols_((imageCols - winX_ + 1) / stepX_),
  spreadX_(16), spreadY_(4),
  imageRows_(imageRows), imageCols_(imageCols),
  weights_(rows_, cols_),
  alpha_(1.0f) {
}

void BandSampler::setPrior(const BandSampler& prior) {
  auto size = windowX() * windowY();
  for(int y = 0; y < imageRows_; y++) {
    for(int x = 0; x < imageCols_; x++) {
      int row = yToRow(y), col = xToCol(x);
      auto tband = this->band(row, col);
      int prow = prior.yToRow(y), pcol = prior.xToCol(x);
      auto pband = prior.band(prow, pcol);
      weights_[tband] += prior.alpha() * prior.weights_[pband] / size;
    }
  }
}

void BandSampler::registerPixel(int y, int x) {
  int row = yToRow(y), col = xToCol(x);
  registerBand(row, col);
}

void BandSampler::registerBand(int row, int col) {
  int 
    cmin = std::max(col - spreadX(), 0),
    cmax = std::min(col + spreadX(), cols() - 1),
    rmin = std::max(row - spreadY(), 0),
    rmax = std::min(row + spreadY(), rows() - 1)
  ;
  tlog(56, "Registering band: row %i of %i spread %i, col %i of %i spread %i",
    row, rows(), spreadY(), col, cols(), spreadX()
  );
  tlog(56, "registration bounds: rows %i to %i, cols %i, to %i, total %i", rmin, rmax, cmin, cmax, weights_.size());
  for(int y = rmin; y <= rmax; y++) {
    for(int x = cmin; x <= cmax; x++) {
      weights_(y, x) = SQ(spreadX()) + SQ(spreadY()) - SQ(x - col) - SQ(y - row);
    }
  }
}

// At first it would seem much simpler to use the std::discrete_distribution
// and just sample directly. But after factoring in replacement and the
// degenerate case of 0 weights, a custom method is preferable
BandSampler::SampledBands BandSampler::sample(int count) {
  // Create a cumulative weight distribution
  VisionTimer::Start(56, "BandSampler::Compute Cumulative");
  Weights cumulative(rows(), cols());
  cumulative[0] = weights_[0];
  for(int i = 1; i < weights_.size(); i++) {
    cumulative[i] = weights_[i] + cumulative[i - 1];
  }
  VisionTimer::Stop(56, "BandSampler::Compute Cumulative");

  // Sample band ids weighted by the cumulative distribution
  SampledIds ids(count);
  auto used = std::vector<bool>(size(), false);
  for(int i = 0; i < ids.size(); i++) {
    // If all weights are 0 this approach won't work, so use a uniform sample
    if(cumulative.back() == 0) {
      sampleUniform(used, ids);
      // sampleUniform samples as many ids as are needed, so it's safe to break out
      break; 
    }
    assert(cumulative.back() > 0);
    // Instantiate a new distribution each iteration...
    uniform_int_distribution<int> distribution(1,cumulative.back());
    auto r = distribution(Random::engine());
    VisionTimer::Start(56, "BandSampler::Binary Search");
    ids[i] = binsearch(cumulative, r);
    VisionTimer::Stop(56, "BandSampler::Binary Search");
    used[ids[i]] = true;
    // ...because we have to update the weights to avoid replacement
    VisionTimer::Start(56, "BandSampler::Adjust Cumulative");
    for(int j = ids[i]; j < cumulative.size(); j++)
      cumulative[j] -= weights_[ids[i]];
    VisionTimer::Stop(56, "BandSampler::Adjust Cumulative");
  }

  // Convert ids to bands sorted by id (row, then column)
  sort(ids.begin(), ids.end());
  SampledBands bands(count);
  for(int i = 0; i < bands.size(); i++) {
    bands[i] = band(ids[i]);
  }
  return bands;
}

void BandSampler::sampleUniform(const std::vector<bool>& used, SampledIds& ids) {
  // Determine which IDs haven't been selected yet
  std::vector<int> options;
  options.reserve(used.size());
  for(int i = 0; i < used.size(); i++)
    if(!used[i])
      options.push_back(i);
  int sampled = this->size() - options.size();
  int needed = ids.size() - sampled;
  assert(sampled >= 0 && needed > 0);

  // Shuffle the unselected IDs and pick as many as are needed
  shuffle(options.begin(), options.end(), Random::engine());
  for(int i = 0; i < needed; i++)
    ids[sampled + i] = options[i];
}

int BandSampler::binsearch(const Weights& weights, int target) {
  assert(weights.back() >= target);
  int i = weights.size() / 2;
  int width = weights.size() / 2;
  if(weights[0] >= target) {
    return 0;
  }
  while(true) {
    assert(abs(width) > 0);
    if(weights[i] < target) {
      width = max(abs(width) / 2,1);
    }
    else if(weights[i - 1] >= target) {
      width = -max(abs(width) / 2,1);
    }
    else break;
    i += width;
  }
  return i;
}
    
std::ostream& operator<<(std::ostream& os, const BandSampler& sampler) {
  os.setf(std::ios_base::fixed, std::ios_base::floatfield);
  os.precision(2);
  os << "Sampler(" << sampler.alpha() << "): ";
  os << "  winY,X: " << sampler.windowY() << "," << sampler.windowX() << ", ";
  os << "stepY,X: " << sampler.stepY() << "," << sampler.stepX() << ", ";
  os << "r: " << sampler.rows() << ", c: " << sampler.cols() << ", ";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Band& band) {
  os << "Band(" << band.row << "," << band.col << "): ";
  os << band.y << "-->" << band.y + band.height << ", ";
  os << band.x << "-->" << band.x + band.width << ", ";
  return os;
}
