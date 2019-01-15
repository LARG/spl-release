
#pragma once

#include <vision/structures/Position.h>
#include <vision/enums/Colors.h>

/// @ingroup vision
struct GoalPostCandidate {

// COMMON TO BOTH

  // Vision/Localization properties 
  uint16_t xi, xf, yi, yf; 
  uint16_t avgX, avgY;
  Position relPosition;     // relative position in world coordinates (calculated from camera matrix's getWorldPosition())

  float width;              // Estimated world width of the post
  bool invalid;
  int invalidIndex;     // For debugging

  float whitePct;

// NEW FIELDS

  uint16_t objectIndex;       // Index into the objects ROI array from FieldEdgeDetector




/// OLD FIELDS

  // Features
  float greenBelowPct;
  int edgeSize;
  int edgeStrength;


  int leftEdgeWidth;    // Width of the edge in pixels
  int rightEdgeWidth;   // Width of the edge in pixels

};


// FOR GOAL DETECTION
//bool sortBlobPixelRatioPredicate(Blob b1, Blob b2);
bool sortPostEdgeStrengthPredicate(GoalPostCandidate b1, GoalPostCandidate b2);
bool sortPostXiPredicate(GoalPostCandidate b1, GoalPostCandidate b2);

bool overlapping(GoalPostCandidate b1, GoalPostCandidate b2);


