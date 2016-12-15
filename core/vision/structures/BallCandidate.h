#ifndef BALL_CANDIDATE_H
#define BALL_CANDIDATE_H

#include <vision/structures/Position.h>
#include <vision/structures/Blob.h>

/// @ingroup vision
struct BallCandidate {
  unsigned int index;
  float centerX;
  float centerY;
  float radius;
  float stddev;
  float width;
  float height;
  float groundDistance;
  float confidence;
  float kwDistanceDiscrepancy;
  float velocity;
  float probability;
  Blob* blob;
  Position relPosition;
  Position absPosition;
  bool valid, best;
  BallCandidate() : blob(NULL), valid(false), best(false) { }
};

#endif
