#include <vision/GoalDetector.h>
#include <iostream>

// Debugging print statements
#ifdef TOOL
  #define DEBUG_OUTPUT true
#else
  #define DEBUG_OUTPUT false
#endif

#if(DEBUG_OUTPUT)
  #define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
  #define DEBUG(x) do { std::cout << __FILENAME__ << "(" << __LINE__ << "): " << x << std::endl; } while (0)
#else
  #define DEBUG(x) do {} while (0)
#endif

static int round_up(int num, int factor);

GoalDetector::GoalDetector(DETECTOR_DECLARE_ARGS) :
  DETECTOR_INITIALIZE {

    //goalTimer.setInterval(30);
    //goalTimer.setMessage("Goal detection");

}

void GoalDetector::detectGoalPosts(const unsigned char *color_table, const std::vector<cv::Point>& field_edge_points) {
  /*cout << "=======================" << endl;
  cout << "Starting Goal Detection" << endl;
  cout << "=======================" << endl;*/

  if (camera_ != Camera::TOP) {
    return;
  }

  //goalTimer.start();

  clear_fields();

  // Detect field boundaries
  vector<cv::Point> hull = constructFieldHull(color_table, field_edge_points);

#ifdef TOOL
  hull_ = hull; // Copy to instance variable so VisionWindow_Draw can draw it
#endif

  if (hull.size() == 0) {
    return;
  }

  // Construct goal post base candidates
  vector<GoalPostBaseCandidate> goal_post_base_candidates = constructBaseCandidates(color_table, hull);

  if (goal_post_base_candidates.size() == 0) {
    return;
  }

  //DEBUG("Goal Base Candidates: " << goal_post_base_candidates.size());

  // Filter goal post base candidates
  filterBaseCandidates(goal_post_base_candidates);

#ifdef TOOL
  goal_post_base_candidates_ = goal_post_base_candidates;
#endif

  int numValid = 0;
  for (auto const &goal_base : goal_post_base_candidates) {
    if (goal_base.invalid) {
      continue;
    }

    int x0 = goal_base.xi;
    int y0 = goal_base.yi;
    int x1 = goal_base.xf;
    int y1 = goal_base.yf;

    int y2 = 0;
    int xleft = ((float) (y1 - y0)*(y0 - y2) / (x1 - x0)) + x0;
    int xright = xleft + (x1 - x0);
    int y3 = 0;
    //cout << x0 << " " << y0 << " " << x1 << " " << y1 << " " << xleft << " " << y2 << " " << xright << " " << y3 << endl;

    // Candidate that follows the slope of the base
    GoalPostCandidate post_candidate_angled;
    post_candidate_angled.x0 = x0;
    post_candidate_angled.y0 = y0;
    post_candidate_angled.x1 = x1;
    post_candidate_angled.y1 = y1;
    post_candidate_angled.x2 = xleft;
    post_candidate_angled.y2 = y2;
    post_candidate_angled.x3 = xright;
    post_candidate_angled.y3 = y3;

    // X value cannot be negative, so invalidate
    if (xleft < 0) {
      post_candidate_angled.invalid = true;
    }

    // Candidate that expands straight upwards
    GoalPostCandidate post_candidate_straight;
    post_candidate_straight.x0 = x0;
    post_candidate_straight.y0 = y0;
    post_candidate_straight.x1 = x1;
    post_candidate_straight.y1 = y1;
    post_candidate_straight.x2 = x0;
    post_candidate_straight.y2 = 0;
    post_candidate_straight.x3 = x1;
    post_candidate_straight.y3 = 0;

    // Expand downwards to green
    int prev_angled_x0 = x0;
    int prev_angled_x1 = x1;
    int prev_y = y0;
    bool angled_done = false;
    bool straight_done = false;
    for (int y = y0; y < min(3*y0, iparams_.height); y += 2) {
      int angled_x0 = ((float) (y1 - y0)*(y0 - y) / (x1 - x0)) + x0;
      int angled_x1 = angled_x0 + (x1 - x0);
      int angled_mid = (angled_x0 + angled_x1) / 2;
      int straight_mid = (x0 + x1) / 2;

      if (!post_candidate_angled.invalid && !angled_done) {
        Color col = xy2color(color_table, angled_mid, y);

        if (col == c_FIELD_GREEN) {
          post_candidate_angled.x0 = prev_angled_x0;
          post_candidate_angled.x1 = prev_angled_x1;
          post_candidate_angled.y0 = prev_y;
          post_candidate_angled.y1 = prev_y + (y1 - y0);
          angled_done = true;
        }
      }

      if (!post_candidate_straight.invalid && !straight_done) {
        Color col = xy2color(color_table, straight_mid, y);

        if (col == c_FIELD_GREEN) {
          post_candidate_straight.y0 = prev_y;
          post_candidate_straight.y1 = prev_y;
          straight_done = true;
        }
      }

      prev_y = y;
      prev_angled_x0 = angled_x0;
      prev_angled_x1 = angled_x1;

      if ((post_candidate_angled.invalid || angled_done) && (post_candidate_straight.invalid || straight_done)) {
        break;
      }
    }

    // Ratio test
    float angle_ratio = (sqrt((float) (post_candidate_angled.x0 - post_candidate_angled.x1)*(post_candidate_angled.x0 - post_candidate_angled.x1) + (post_candidate_angled.y0 - post_candidate_angled.y1)*(post_candidate_angled.y0 - post_candidate_angled.y1)) / sqrt((float) (post_candidate_angled.x0 - post_candidate_angled.x2)*(post_candidate_angled.x0 - post_candidate_angled.x2) + (post_candidate_angled.y0 - post_candidate_angled.y2)*(post_candidate_angled.y0 - post_candidate_angled.y2)));
    float straight_ratio = (sqrt((float) (post_candidate_straight.x0 - post_candidate_straight.x1)*(post_candidate_straight.x0 - post_candidate_straight.x1) + (post_candidate_straight.y0 - post_candidate_straight.y1)*(post_candidate_straight.y0 - post_candidate_straight.y1)) / sqrt((float) (post_candidate_straight.x0 - post_candidate_straight.x2)*(post_candidate_straight.x0 - post_candidate_straight.x2) + (post_candidate_straight.y0 - post_candidate_straight.y2)*(post_candidate_straight.y0 - post_candidate_straight.y2)));
    if (angle_ratio > 0.6 || angle_ratio < 0.1) {
      post_candidate_angled.invalid = true;
    }
    if (straight_ratio > 0.6 || straight_ratio < 0.1) {
      post_candidate_straight.invalid = true;
    }

    // Scan candidate region for both angled and straight
    int min_y = min({post_candidate_angled.y0, post_candidate_angled.y1,
                     post_candidate_straight.y0, post_candidate_straight.y1});
    int count_angled = 0;
    int count_straight = 0;
    int xstep = 2;
    int ystep = 2;
    // Intensities at horizontal 1/3, 1/2, and 2/3 points
    array<float, 3> prev_angled_intensities = {-1, -1, -1};
    array<float, 3> prev_straight_intensities = {-1, -1, -1};
    for (; y2 < min_y; y2 += ystep) {
      xleft = ((float) (y1 - y0)*(y0 - y2) / (x1 - x0)) + x0;
      xright = xleft + (x1 - x0);

      // x values to check intensity
      int angled_mid = round_up((xleft + xright)/2, xstep);
      int straight_mid = round_up((x0 + x1)/2, xstep);
      int angled_left = round_up((0.666*xleft + 0.333*xright), xstep);
      int straight_left = round_up((0.666*x0 + 0.333*x1), xstep);
      int angled_right = round_up((0.333*xleft + 0.666*xright), xstep);
      int straight_right = round_up((0.333*x0 + 0.666*x1), xstep);

      int xmin = post_candidate_angled.invalid ? round_up(x0, xstep) : round_up(min(xleft, x0), xstep);
      int xmax = post_candidate_angled.invalid ? x1 : max(xright, x1);
      for (int i = xmin; i <= xmax; i += xstep) {
        Color col = xy2color(color_table, i, y2);
        int y, u, v;

        int angled_intensities_index = -1;
        int straight_intensities_index = -1;
        if (i == angled_left) {
          angled_intensities_index = 0;
          xy2yuv(i, y2, y, u, v);
        } else if (i == angled_mid) {
          angled_intensities_index = 1;
          xy2yuv(i, y2, y, u, v);
        } else if (i == angled_right) {
          angled_intensities_index = 2;
          xy2yuv(i, y2, y, u, v);
        }

        if (i == straight_left) {
          straight_intensities_index = 0;
          xy2yuv(i, y2, y, u, v);
        } else if (i == straight_mid) {
          straight_intensities_index = 1;
          xy2yuv(i, y2, y, u, v);
        } else if (i == straight_right) {
          straight_intensities_index = 2;
          xy2yuv(i, y2, y, u, v);
        }

        // Angled analysis
        if (!post_candidate_angled.invalid && i >= xleft && i <= xright) {
          if (angled_intensities_index >= 0) {
            if (prev_angled_intensities[angled_intensities_index] < 0) {
              prev_angled_intensities[angled_intensities_index] = y;
            } else {
              float diff = abs(y - prev_angled_intensities[angled_intensities_index]);

              //post_candidate_angled.diffs.push_back(make_pair(make_pair(i, y2), diff));
              if (diff > 45) {
                //DEBUG("Invalidated due to diff of " << diff);
                // TODO: break if invalid
                post_candidate_angled.diff = diff;
                post_candidate_angled.difflocX = i;
                post_candidate_angled.difflocY = y2;
                post_candidate_angled.invalid = true;
              }

              prev_angled_intensities[angled_intensities_index] = y;
            }
          }

          count_angled++;
          if (col == c_WHITE) {
            post_candidate_angled.whitePct++;
          } else if (col == c_ROBOT_WHITE) {
            post_candidate_angled.whitePct+=0.5;
          }
        }

        // Straight analysis
        if (!post_candidate_straight.invalid && i >= x0 && i <= x1) {
          if (straight_intensities_index >= 0) {
            if (prev_straight_intensities[straight_intensities_index] < 0) {
              prev_straight_intensities[straight_intensities_index] = y;
            } else {
              float diff = abs(y - prev_straight_intensities[straight_intensities_index]);

              //post_candidate_straight.diffs.push_back(make_pair(make_pair(i, y2), diff));
              if (diff > 45) {
                //DEBUG("Invalidated due to diff of " << diff);
                // TODO: break if invalid
                post_candidate_straight.diff = diff;
                post_candidate_straight.difflocX = i;
                post_candidate_straight.difflocY = y2;
                post_candidate_straight.invalid = true;
              }

              prev_straight_intensities[straight_intensities_index] = y;
            }
          }

          count_straight++;
          if (col == c_WHITE) {
            post_candidate_straight.whitePct++;
          } else if (col == c_ROBOT_WHITE) {
            post_candidate_straight.whitePct+=0.5;
          }
        }
      }

      if (post_candidate_angled.invalid && post_candidate_straight.invalid) {
        break;
      }
    }
    post_candidate_angled.whitePct = (count_angled > 0) ? post_candidate_angled.whitePct / count_angled : 0.;
    post_candidate_straight.whitePct = (count_straight > 0) ? post_candidate_straight.whitePct / count_straight : 0.;

    if (post_candidate_angled.whitePct < 0.4) {
      post_candidate_angled.invalid = true;
    }
    if (post_candidate_straight.whitePct < 0.4) {
      post_candidate_straight.invalid = true;
    }

    if (!post_candidate_straight.invalid || !post_candidate_angled.invalid) {
      numValid++;
    }

    if (post_candidate_straight.whitePct >= post_candidate_angled.whitePct && !post_candidate_straight.invalid) {
      goal_post_candidates_.push_back(post_candidate_straight);
      valid_goal_post_candidates_.push_back(post_candidate_straight);
    } else {
      goal_post_candidates_.push_back(post_candidate_angled);
      if (!post_candidate_angled.invalid) {
        valid_goal_post_candidates_.push_back(post_candidate_angled);
      }
    }

  }

  if (numValid > 2) {
    goal_post_candidates_.clear();
    valid_goal_post_candidates_.clear();
  }

//  end = std::chrono::high_resolution_clock::now();
//  time_ms = end - start;

//  cout << "Goal Post Candidate Time: " << time_ms.count() << " milliseconds" << endl;

  if (valid_goal_post_candidates_.size() == 0) {
    return;
  }

  if (numValid == 1) {
    auto &cand = valid_goal_post_candidates_[0];
    int avgX = (cand.x0 + cand.x1) / 2;
    int avgY = (cand.y0 + cand.y2) / 2;
    int maxY = max(cand.y0, cand.y1);
    auto pos = cmatrix_.getWorldPosition(avgX, maxY);

    float d1 = cmatrix_.groundDistance(pos);
    float b1 = cmatrix_.bearing(pos);
    float e1 = cmatrix_.elevation(pos);

    int postIndex = WO_UNKNOWN_GOALPOST;
    setGoalObject(postIndex, d1, b1, e1, avgX, maxY, 1, -1);
  } else {
    auto &cand1 = valid_goal_post_candidates_[0];
    auto &cand2 = valid_goal_post_candidates_[1];
    int avgX1 = (cand1.x0 + cand1.x1) / 2;
    int avgY1 = (cand1.y0 + cand1.y2) / 2;
    int maxY1 = max(cand1.y0, cand1.y1);
    int avgX2 = (cand2.x0 + cand2.x1) / 2;
    int avgY2 = (cand2.y0 + cand2.y2) / 2;
    int maxY2 = max(cand2.y0, cand2.y1);

    auto pos1 = cmatrix_.getWorldPosition(avgX1, avgY1);
    float d1 = cmatrix_.groundDistance(pos1);
    float b1 = cmatrix_.bearing(pos1);
    float e1 = cmatrix_.elevation(pos1);

    auto pos2 = cmatrix_.getWorldPosition(avgX2, avgY2);
    float d2 = cmatrix_.groundDistance(pos2);
    float b2 = cmatrix_.bearing(pos2);
    float e2 = cmatrix_.elevation(pos2);

    // Populate both world objects

    float bearing = (b1 + b2) / 2, elevation = (e1 + e2) / 2;
    float confidence = 1, confidence1 = 1, confidence2 = 1;

    // Decide which post is left and which post is right
    auto left = pos1, right = pos2;
    int post1 = WO_UNKNOWN_LEFT_GOALPOST;
    int post2 = WO_UNKNOWN_RIGHT_GOALPOST;
    if (avgX2 < avgX1) {
      post1 = WO_UNKNOWN_RIGHT_GOALPOST;
      post2 = WO_UNKNOWN_LEFT_GOALPOST;
      left = pos2;
      right = pos1;
    }

    // TODO: See formGoal for distance calculations 
    float distancePosts = estimateGoalDistanceByPosts(left, right);
    float distance = distancePosts;

    // Set Goals and Goal Posts
    setGoalObject(WO_UNKNOWN_GOAL, distance, bearing, elevation,
        (avgX1 + avgX2) / 2,
        (avgY1 + avgY2) / 2,
        confidence, -1);
    setGoalObject(post1, d1, b1, e1,
        avgX1, maxY1,
        confidence1, -1);
    setGoalObject(post2, d2, b2, e2,
        avgX2, maxY2,
        confidence2, -1);
  }

  //goalTimer.stop();
  //goalTimer.printAtInterval();

  /*cout << "=======================" << endl;
  cout << "Finished Goal Detection" << endl;
  cout << "=======================" << endl;*/
}

/* Clear class instance variables */
void GoalDetector::clear_fields() {
#ifdef TOOL
  field_boundary_color_.clear();
  hull_.clear();
  goal_post_base_candidates_.clear();
#endif
  goal_post_candidates_.clear();
  valid_goal_post_candidates_.clear();
}

/*
 * Constructs convex hull of the field boundary
 * Note: y-values from returned vector need to be obtained by iparams_.height - y
 */
vector<cv::Point> GoalDetector::constructFieldHull(const unsigned char *color_table, const std::vector<cv::Point> &field_edge_points) {
  vector<cv::Point> hull;

  // Convex Hull
  if (field_edge_points.size() > 1) {
    cv::convexHull(field_edge_points, hull, true);
  }

  return hull;
}

/*
 * Constructs goal post base candidates by traversing under the field boundary
 * for white blobs
 */
vector<GoalPostBaseCandidate> GoalDetector::constructBaseCandidates(const unsigned char *color_table, const vector<cv::Point> &hull) {
  vector<GoalPostBaseCandidate> candidates;

  int prev_x = -1;
  int prev_y = -1;
  for (auto const &hull_point : hull) {
    int next_y = iparams_.height - hull_point.y;
    int next_x = hull_point.x;
    if (prev_x < 0) {
      prev_x = next_x;
      prev_y = next_y;
      continue;
    }

    // Upper Hull
    if (next_x < prev_x) {
      break;
    }

    // Scan delta lines below the detected field boundary
    // to find goal post base candidates
    int initial_delta = 36;
    for (int delta = initial_delta; delta >= 0; delta -= 2) {
      int x = prev_x;
      while (x < next_x) {
        // Linear interpolation
        int calc_y = (int) (prev_y + (next_y - prev_y) * ((float) (x - prev_x) / (next_x - prev_x)));
        //cout << "prev: (" << prev_x << ", " << prev_y << ") now: (" << x << ", " << y << ") next: (" << next_x << ", " << next_y << ")" << endl;

        int y = calc_y + delta;
        int lower_y = prev_y + delta;
        int upper_y = next_y + delta;

        // Find first white pixel
        Color col = xy2color(color_table, x, y);
        for (; x < next_x && col != c_WHITE; x += 3) {
          y = (int) (lower_y + (upper_y - lower_y) * ((float) (x - prev_x) / (next_x - prev_x)));
          col = xy2color(color_table, x, y);
        }
        int x0 = x;
        int y0 = y;

        // Continue until a green pixel is encountered
        int xcount = 0;
        int whitecount = 0;
        col = xy2color(color_table, x, y);
        for (; x < next_x && col != c_FIELD_GREEN; x += 3, xcount++) {
          y = (int) (lower_y + (upper_y - lower_y) * ((float) (x - prev_x) / (next_x - prev_x)));
          col = xy2color(color_table, x, y);

          if (col == c_WHITE || col == c_ROBOT_WHITE) {
            whitecount++;
          }
        }
        int x1 = x;
        int y1 = y;

        float whitepercentage = 0.0;
        if (xcount > 0) {
          whitepercentage = (float) whitecount / xcount;
        }

        // Only add candidate is white percentage and size is sufficient
        //cout << "White Percentage: " << whitepercentage << endl;
        if (x1 - x0 > 16 && whitepercentage > 0.6 && x1 - x0 < 200) {
          GoalPostBaseCandidate cand;
          cand.xi = x0;
          cand.yi = y0;
          cand.xf = x1;
          cand.yf = y1;

          candidates.push_back(cand);
          break;
        }
      }
    }

    prev_x = next_x;
    prev_y = next_y;
  }

  auto compare_func = [](const GoalPostBaseCandidate &ca, const GoalPostBaseCandidate &cb) {
    return ca.xi < cb.xi;
  };
  if (candidates.size() > 1) {
    sort(candidates.begin(), candidates.end(), compare_func);
  }

  return candidates;
}

/*
 * Filters goal post base candidates
 */
void GoalDetector::filterBaseCandidates(vector<GoalPostBaseCandidate> &candidates) {
  // Filter goal post base candidates
  for (unsigned i = 0; i < candidates.size() - 1; i++) {
    auto &base = candidates[i];

    // Compare candidate to candidates on the right
    for (unsigned j = i + 1; j < candidates.size(); j++) {
      auto &next_base = candidates[j];
      int w1 = base.xf - base.xi;
      int w2 = next_base.xf - next_base.xi;

      if (GoalPostBaseCandidate::is_contained(base, next_base)) {
        //cout << "Invalidating next_base" << endl;
        next_base.invalid = true;
        i++;
      } else if (GoalPostBaseCandidate::is_contained(next_base, base)) {
        //cout << "Invalidating base" << endl;
        base.invalid = true;
        break;
      } else if (GoalPostBaseCandidate::almost_identical(base, next_base)) {
        if (w1 > w2) {
          base.invalid = true;
          break;
        } else {
          next_base.invalid = true;
          i++;
        }
      } else if (GoalPostBaseCandidate::is_overlapping(base, next_base)) {
        // Invalidate base least closest to width 60
        if (abs(w1 - 60) <= abs(w2 - 60)) {
          next_base.invalid = true;
          i++;
        } else {
          base.invalid = true;
          break;
        }
      } else {
        break;
      }
    }
  }
}

void GoalDetector::setGoalObject(int goalIndex, float distance, float bearing, float elevation, int centerX, int centerY, float confidence, int lineIndex) {
  vblocks_.world_object->objects_[goalIndex].seen = true;
  vblocks_.world_object->objects_[goalIndex].visionBearing = bearing;
  vblocks_.world_object->objects_[goalIndex].visionElevation = elevation;
  vblocks_.world_object->objects_[goalIndex].visionDistance = distance;
  vblocks_.world_object->objects_[goalIndex].imageCenterX = centerX;
  vblocks_.world_object->objects_[goalIndex].imageCenterY = centerY;
  vblocks_.world_object->objects_[goalIndex].frameLastSeen = vblocks_.frame_info->frame_id;
  // HACK HERE  
  vblocks_.world_object->objects_[goalIndex].visionConfidence = 1.0; //confidence;
  //  vblocks_.world_object->objects_[goalIndex].fieldLineIndex = lineIndex;
  vblocks_.world_object->objects_[goalIndex].fromTopCamera = (camera_ == Camera::TOP);
}

float GoalDetector::estimateGoalDistanceByPosts(Position left, Position right) {
  float kdist = (cmatrix_.groundDistance(left) + cmatrix_.groundDistance(right)) / 2.0f;
  float apparentWidth = (left - right).abs();

  // When we assume the goal is farther away, we over-estimate its world width.
  float cdist = kdist * GOAL_WIDTH / apparentWidth;
  tlog(35, "Both posts found, computing distance from apparent width: %2.f ( / %2.f = %2.2f), kdist: %2.f, corrected dist: %2.f", apparentWidth, GOAL_WIDTH, GOAL_WIDTH / apparentWidth, kdist, cdist);
  return cdist;
}

static int round_up(int num, int factor) {
  return num + factor - 1 - (num - 1) % factor;
}
