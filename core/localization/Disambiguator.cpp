#include <localization/Disambiguator.h>
#include <memory/TextLogger.h>
#include <memory/WorldObjectBlock.h>
#include <memory/RobotStateBlock.h>
#include <common/Field.h>
#include <localization/EnumeratedSolver.h>
#include <math/HungarianSolver.h>
#include <localization/Logging.h>

using AssignmentSolver = HungarianSolver;

#define getCandidate(i) ObjectCandidate(sloc_, sorient_, \
    cache_.world_object->objects_[i], query, tlogger_\
)

#define LVL 49
#define VLVL 51

#define THROWOUT_COST 100000
#define IGNORE_COST 100001

using namespace std;
using namespace Eigen;
  
ObjectCandidate::ObjectCandidate(Point2D sloc, float sorient, WorldObject obj, const WorldObject& query, TextLogger* tlogger) : obj(obj), tlogger_(tlogger) {
  if(query.isLine()) {
    auto tl = obj.lineLoc, ql = query.lineLoc;
    auto angle = ql.getSignedAngleToLine(tl);
    auto rotated = ql;
    rotated.rotate(sloc, angle);    // Rotates the line(segment?) using sloc (self location) as the origin, and by the given angle. 
    float ddiff = fabs(angle);
    tlog(VLVL, "ObjectCandidate %s distance: %s vs %s", getName(obj.type), tl, ql);
    tlog(VLVL, "rotated about sloc %s, angle %2.2f: %s", sloc, angle * RAD_T_DEG, rotated);
    tlog(VLVL, "angle diff: %2.2f", ddiff * RAD_T_DEG);
    ql.computeCenter(); tl.computeCenter(); rotated.computeCenter();
    float radiusDist = fabs(ql.center.getMagnitude() - CIRCLE_RADIUS);
    float qlen = ql.length(), tlen = tl.length();
    float cdist = tl.getDistanceTo(ql.center);            // Distance from ground truth line to center of query/detected line
    float rcdist = tl.getDistanceTo(rotated.center);      // Distance from ground truth line to rotated query line. I think this tries to calculate distance correcting for angle differences
    float cost1 = powf(cdist, 1.2f);                    
    float cost2 = powf(rcdist, 1.4f);                     // Increasing this because if two lines are very close, we should trust position more than slight differences in angle?
    assert(tlen > 0);
    float cost3 = powf(max(1.0f, qlen / tlen), 2);     // Ratio of seen query line to ground truth line. So if we don't detect the whole line, we give cost 1, if we detect a line larger than expected, then the cost increases quadratically 
    float cost4 = max(powf(fabs(ddiff) * RAD_T_DEG / 10.f,1.3f), 1.f);      // Penalizes by the angle difference
    cost = (cost1 + cost2) * (cost3 * cost4);
    tlog(VLVL, "c1: %2.4f, c2: %2.4f, c3: %2.4f, c4: %2.4f, qlen: %2.2f, tlen: %2.2f, cdist: %2.2f, rcdist: %2.2f, ddiff: %2.2f, far line  distances", cost1, cost2, cost3, cost4, qlen, tlen, cdist, rcdist, ddiff * RAD_T_DEG);
    tlog(VLVL, "final cost: %2.2f", cost);
  } else if(query.isGoal()) {
    tlog(VLVL, "computing cost for %s vs %s", getName(obj.type), getName(query.type));
    float tbearing = sloc.getBearingTo(obj.loc, sorient);
    float bearingDiff = normalizeAngle(query.visionBearing - tbearing);
    float distDiff = query.loc.getDistanceTo(obj.loc);
    cost = max(1.0, bearingDiff * RAD_T_DEG / 5) * distDiff / 3;
    tlog(VLVL, "qbearing: %2.2f, tbearing: %2.2f, dist: %2.f, cost: %2.f", query.visionBearing * RAD_T_DEG, tbearing * RAD_T_DEG, distDiff, cost);
  }
  else {
    tlog(VLVL, "query %s at %2.f,%2.f, test %s at %2.f,%2.f", 
      getName(query.type), query.loc.x, query.loc.y,
      getName(obj.type), obj.loc.x, obj.loc.y
    );
    cost = obj.loc.getDistanceTo(query.loc);
  }
}

Disambiguator* Disambiguator::instance_ = nullptr;

Disambiguator::Disambiguator(BASE_DECLARATIONS) : LocalizationBase(BASE_ARGS) {
  merge_utility_ = std::make_unique<PairwiseCostMergeUtility<WorldObject,float>>();
  if(instance_ == nullptr)
    instance_ = this;
}

vector<WorldObject> Disambiguator::pruneNetDetections(const vector<WorldObject>& queries) {
  vector<WorldObject> pruned;
  bool inGoalBox = false;
  if(fabs(sloc_.x) > HALF_FIELD_X - PENALTY_X && fabs(sloc_.y) < PENALTY_Y/2)
    inGoalBox = true;
  tlog(VLVL, "pruning net detections, self: %s, in goal box: %i", sloc_, inGoalBox);
  if(!inGoalBox) return queries;
  for(const auto& q : queries) {
    auto ql = q.lineLoc;
    float endlineThresh = 500;
    if(fabs(ql.start.x) > HALF_GRASS_X + endlineThresh && fabs(ql.end.x) > HALF_GRASS_X + endlineThresh && fabs(ql.center.x) > HALF_GRASS_X + endlineThresh && fabs(ql.center.y) < 1000) {
      tlog(VLVL, "threw out %s at %s during goal net check", getName(q.type), ql);
      continue;
    }
    pruned.push_back(q);
  }
  return pruned;
}

vector<WorldObject> Disambiguator::pruneCirclePieces(const vector<WorldObject>& queries) {
  vector<WorldObject> pruned;
  for(const auto& q : queries) {
    auto ql = q.lineLoc;
    Point2D perpCenter(-ql.center.y, ql.center.x);
    float circleSlopeDiff = fabs(perpCenter.getDirection() - ql.getDirection());
    if(circleSlopeDiff > M_PI / 2) circleSlopeDiff -= M_PI;
    if(fabs(ql.center.getMagnitude() - CIRCLE_RADIUS) < 250 && ql.length() < 500 && fabs(circleSlopeDiff * RAD_T_DEG) < 10) {
      tlog(VLVL, "threw out %s at %s during center circle piece check", getName(q.type), ql);
      continue;
    }
    pruned.push_back(q);
  }
  return pruned;
}

vector<int> Disambiguator::findCandidateTypes(const WorldObject& query) {
  vector<int> types;
  if(query.isUnknownLine()) {
    for(int i = LINE_OFFSET; i < LINE_OFFSET + NUM_LINES; i++) {
      if(!config_.use_field_edges && WorldObject::isEdgeLine(i)) continue;
      types.push_back(i);
    }
  }
  else if (query.type == WO_UNKNOWN_GOAL) {
    types.push_back(WO_OWN_GOAL);
    types.push_back(WO_OPP_GOAL);
  }
  else if (query.type == WO_UNKNOWN_RIGHT_GOALPOST) {
    types.push_back(WO_OWN_RIGHT_GOALPOST);
    types.push_back(WO_OPP_RIGHT_GOALPOST);
  }
  else if (query.type == WO_UNKNOWN_LEFT_GOALPOST) {
    types.push_back(WO_OWN_LEFT_GOALPOST);
    types.push_back(WO_OPP_LEFT_GOALPOST);
  }
  else if (query.type == WO_UNKNOWN_GOALPOST) {
    types.push_back(WO_OWN_LEFT_GOALPOST);
    types.push_back(WO_OPP_LEFT_GOALPOST);
    types.push_back(WO_OWN_RIGHT_GOALPOST);
    types.push_back(WO_OPP_RIGHT_GOALPOST);
  }
  else if (config_.mix_line_intersections && query.isUnknownIntersection()) {
    for(int i = T_INT_OFFSET; i < T_INT_OFFSET + NUM_T_INTS; i++) {
      if(!config_.use_field_edges && WorldObject::isEdgeIntersection(i)) continue;
      types.push_back(i);
    }
    for(int i = L_INT_OFFSET; i < L_INT_OFFSET + NUM_L_INTS; i++) {
      if(!config_.use_field_edges && WorldObject::isEdgeIntersection(i)) continue;
      types.push_back(i);
    }
  }
  else if (!config_.mix_line_intersections && query.isUnknownT()) {
    for(int i = T_INT_OFFSET; i < T_INT_OFFSET + NUM_T_INTS; i++) {
      if(!config_.use_field_edges && WorldObject::isEdgeIntersection(i)) continue;
      types.push_back(i);
    }
  }
  else if (!config_.mix_line_intersections && query.isUnknownL()) {
    for(int i = L_INT_OFFSET; i < L_INT_OFFSET + NUM_L_INTS; i++) {
      if(!config_.use_field_edges && WorldObject::isEdgeIntersection(i)) continue;
      types.push_back(i);
    }
  }
  else if (query.isUnknownPenaltyCross()) {
    types.push_back(WO_OPP_PENALTY_CROSS);
    types.push_back(WO_OWN_PENALTY_CROSS);
  }
  return types;
}

/** Generates object candidates for unknown/ambiguous objects
  E.g. given a query for WO_UNKNOWN_FIELD_LINE_1, this could correspond to any of the actual lines on the field (such as WO_TOP_SIDE_LINE, WO_OWN_FIELD_EDGE, etc).
 */
vector<ObjectCandidate> Disambiguator::findCandidates(const WorldObject& query) {
  vector<ObjectCandidate> candidates;
  auto types = findCandidateTypes(query);
  for(auto t : types)
    candidates.push_back(getCandidate(t));
  tlog(48, "Found %d candidates for query", candidates.size());
  return candidates;
}

void Disambiguator::sortCandidates(const WorldObject& query, vector<ObjectCandidate>& candidates) {
  std::sort(candidates.begin(), candidates.end(),
    [](ObjectCandidate a, ObjectCandidate b) {
      return a.cost < b.cost;
    }
  );
}

float Disambiguator::distance(const WorldObject* left, const WorldObject* right) {
  auto tlogger_ = instance_->tlogger_;
  if(left->isLine()) {
    auto ll = left->visionLine, rl = right->visionLine;
    float ddiff = fabs(ll.getDirection() - rl.getDirection());
    if(ddiff > M_PI / 2) ddiff -= M_PI;
    // If line angles differ by more than 30 degrees, return a large dist
    tlog(48, "angle distance: %s vs %s", ll, rl);
    tlog(48, "ldir: %2.2f, rdir: %2.2f, angle diff: %2.2f", ll.getDirection() * DEG_T_RAD, rl.getDirection() * DEG_T_RAD, ddiff * DEG_T_RAD);
    if(fabs(ddiff) > M_PI / 6) return THROWOUT_COST; 
    rl.computeCenter();
    // Get the distance from the extended left line to the center of the right segment
    // If these are two parts of the same line then this will be close to 0.
    float dist = ll.getDistanceToPoint(rl.center);
    tlog(48, "center dist: %2.2f", dist);
    return dist;
  } else {
    return left->loc.getDistanceTo(right->loc);
  }
}

WorldObject* Disambiguator::mergeIntersections(const WorldObject* left, const WorldObject* right) {
  auto merged = new WorldObject();
  merged->type = min(left->type, right->type);
  merged->loc = (left->loc + right->loc) / 2;
  return merged;
}

WorldObject* Disambiguator::mergeLines(const WorldObject* left, const WorldObject* right) {
  auto merged = new WorldObject();
  merged->type = min(left->type, right->type);
  vector<LineSegment> mlines {
    LineSegment(left->visionLine.start,right->visionLine.start),
    LineSegment(left->visionLine.start,right->visionLine.end),
    LineSegment(left->visionLine.end,right->visionLine.start),
    LineSegment(left->visionLine.end,right->visionLine.end)
  };
  std::sort(mlines.begin(), mlines.end(),
    [](const LineSegment& a, const LineSegment& b) {
      return a.length() > b.length();
    }
  );
  merged->visionLine = mlines[0];
  mlines = {
    LineSegment(left->lineLoc.start,right->lineLoc.start),
    LineSegment(left->lineLoc.start,right->lineLoc.end),
    LineSegment(left->lineLoc.end,right->lineLoc.start),
    LineSegment(left->lineLoc.end,right->lineLoc.end)
  };
  std::sort(mlines.begin(), mlines.end(),
    [](const LineSegment& a, const LineSegment& b) {
      return a.length() > b.length();
    }
  );
  merged->lineLoc = mlines[0];
  return merged;
}

std::vector<WorldObject> Disambiguator::pruneDuplicates(const std::vector<WorldObject>& objects, function<WorldObject*(const WorldObject*,const WorldObject*)> mergeFunction, function<bool(const WorldObject&)> predicate) {
  float threshold = 100;
  vector<WorldObject*> pobjects;
  for(const auto& wo : objects)
    if(predicate(wo))
      pobjects.push_back(new WorldObject(wo));
  // This tries to merge objects that match the predicate (at the time of this writing, either isLine() or isUnknownIntersection())
  tlog(VLVL, "merging %i objects", pobjects.size());
  // Pass in the objects to be merged, -1 means use the threshold, the function that says how to merge, and the cost function for merging. Returns a vector of merged objects
  auto pmerged = merge_utility_->merge_items(pobjects, -1, threshold, mergeFunction, Disambiguator::distance);
  vector<WorldObject> mobjects;
  for(auto p : pmerged) {
    mobjects.push_back(WorldObject(*p));
    delete p;
  }
  for(auto wo : objects)
    if(!predicate(wo))
      mobjects.push_back(wo);
  tlog(VLVL, "Returning %i merged objects", mobjects.size());
  return mobjects;
}

std::vector<WorldObject> Disambiguator::pruneDuplicateLines(const std::vector<WorldObject>& objects) {
  tlog(VLVL, "pruning duplicate lines from %i objects", objects.size());
  return pruneDuplicates(objects, mergeLines, [](const WorldObject& wo) { return wo.isLine(); });
}

std::vector<WorldObject> Disambiguator::pruneDuplicateIntersections(const std::vector<WorldObject>& objects) {
  tlog(VLVL, "pruning duplicate intersections from %i objects", objects.size());
  return pruneDuplicates(objects, mergeIntersections, [](const WorldObject& wo) { return wo.isUnknownIntersection(); });
}

void Disambiguator::addCandidate(const WorldObject& query, ObjectCandidate& cand, vector<ObjectSet>& objectSets, bool* used) {
  if(used && !cand.obj.isLine() && used[cand.obj.type]) {
    printf("ERROR: candidate %s requested for re-add\n", getName(cand.obj.type));
    throw -1;
  }
  if(used) used[cand.obj.type] = true;
  cand.obj.relPos = query.relPos;
  cand.obj.visionBearing = query.visionBearing;
  cand.obj.visionDistance = query.visionDistance;
  cand.obj.visionLine = query.visionLine;
  cand.obj.utype = query.type;
  for(auto& objects : objectSets)
    objects.push_back(cand.obj);
}

void Disambiguator::selectCandidates(const WorldObject& query, vector<ObjectCandidate>& candidates, vector<ObjectSet>& objectSets, bool* used, const set<WorldObjectType>& affinities) {
  bool success = false;
  float maxGoalCost = 2500;
  // For now, just check goal posts. In the future, allow multiple types of affinities.
  if(query.type == WO_UNKNOWN_GOALPOST) {
    if(affinities.size() > 0) {
      for(auto cand : candidates) {
        if(!cand.obj.isLine() && used[cand.obj.type]) continue;
        if(affinities.find(cand.obj.type) != affinities.end()) {
          if(cand.cost > maxGoalCost) {
            tlog(VLVL, "skipping goal due to cost: %2.f", maxGoalCost);
            continue;
          }
          addCandidate(query, cand, objectSets, used);
          return;
        }
      }
      tlog(VLVL, "THROWOUT: no candidates added for goal %s", getName(query.type));
      thrown_out_++;
    } else { // Create two copies of object sets, add cand 1 to the first and cand 2 to the second, recombine
      tlog(VLVL, "checking goal costs: %2.2f, %2.2f", candidates[0].cost, candidates[1].cost);
      auto c1 = objectSets; auto c2 = objectSets;
      if(candidates[0].cost < maxGoalCost && candidates[1].cost < maxGoalCost) {
        tlog(VLVL, "added post as both posts with candidate sets");
        addCandidate(query, candidates[0], c1, used);
        addCandidate(query, candidates[1], c2, used);
        objectSets.clear();
        objectSets.insert(objectSets.end(), c1.begin(), c1.end());
        objectSets.insert(objectSets.end(), c2.begin(), c2.end());
      } 
      else if(candidates[0].cost < maxGoalCost) {
        tlog(VLVL, "added post as %s", getName(candidates[0].obj.type));
        addCandidate(query, candidates[0], objectSets, used);
      }
      else if(candidates[1].cost < maxGoalCost) {
        tlog(VLVL, "added post as %s", getName(candidates[1].obj.type));
        addCandidate(query, candidates[1], objectSets, used);
      } else {
        tlog(VLVL, "THROWOUT: goal post because of costs %2.f, %2.f > %2.f", candidates[0].cost, candidates[1].cost, maxGoalCost);
        thrown_out_++;
      }
      return;
    }
  }
  else {
    for(auto cand : candidates) {
      if(!cand.obj.isLine() && used[cand.obj.type]) continue;
      if(cand.obj.isGoal()) {
        if(cand.cost > maxGoalCost) {
          tlog(VLVL, "skipping goal due to cost: %2.f", maxGoalCost);
          continue;
        }
      }
      addCandidate(query, cand, objectSets, used);
      return;
    }
    tlog(VLVL, "THROWOUT: no candidates added for goal %s", getName(query.type));
    thrown_out_++;
  }
}

/** Queries is a vector of ambiguous world objects 
 
  This function tries to match ambiguous world objects to known world objects, using a cost function and assignment solver.
 */
void Disambiguator::matchSubset(vector<WorldObject> queries, vector<ObjectSet>& objectSets) {
  if(queries.size() == 0) return;
  auto types = findCandidateTypes(queries[0]);
  if(queries[0].isLine()) {
    queries = pruneCirclePieces(queries);
    queries = pruneNetDetections(queries);
  }
  if(queries.size() == 0) return;
  MatrixXf costs(queries.size(), types.size());
  // Generate a cost matrix where i is the query, j is the possible matching, and (i,j) is the cost of that matching
  for(int i = 0; i < queries.size(); i++) {
    const auto& query = queries[i];
    for(int j = 0; j < types.size(); j++) {
      auto t = types[j];
      auto candidate = getCandidate(t);
      costs(i,j) = candidate.cost;
      tlog(LVL, "cost(%i,%i): %2.2f", i, j, candidate.cost);
    }
  }
  // Solver for the best matching, and throwout(?) costs that are more than maxCost
  float maxCost = 20000;
  AssignmentSolver solver(maxCost);
  auto assignments = solver.solve(costs);
  tlog(LVL, "THROWOUT: %i items from the assignment solver due to max cost of %2.f", queries.size() - solver.count(), maxCost);
  thrown_out_ += (queries.size() - solver.count());
  if(solver.count() == 0) return;
  // Debugging printouts/logging
  for(int i = 0; i < assignments.size(); i++) {
    tlog(LVL, "ASSIGNMENT[%i]: %i", i, assignments[i]);
    std::stringstream ss;
    ss << "COSTS[%i]: ";
    for(int j = 0; j < costs.cols(); j++) {
      ss << "[" << j << "]=" << costs(i,j) << ",";
    }
    tlog(LVL, ss.str().c_str(), i);
  }

  tlog(LVL, "produced %i assignments", assignments.size());
  for(int i = 0; i < assignments.size(); i++) {
    if(assignments[i] == -1) continue;
    if(costs(i, assignments[i]) >= maxCost) continue;
    const auto& query = queries[i];
    auto type = types[assignments[i]];
    auto candidate = getCandidate(type);
    tlog(LVL, "ASSIGNMENT[%i]: %i", i, assignments[i]);
    tlog(LVL, "assigned %s to %s, cost %2.2f", getName(query.type), getName((WorldObjectType)type), costs(i, assignments[i]));
    addCandidate(query, candidate, objectSets, NULL);
  }
}

void Disambiguator::matchObjects(const vector<WorldObject>& queries, vector<ObjectSet>& objectSets) {
  vector<WorldObject> lines, ints, ts, ls;
  for(auto q : queries) {
    if(q.isUnknownLine()) lines.push_back(q);
    else if(config_.mix_line_intersections && q.isUnknownIntersection())
      ints.push_back(q);
    else if(!config_.mix_line_intersections && q.isUnknownT()) ts.push_back(q);
    else if(!config_.mix_line_intersections && q.isUnknownL()) ls.push_back(q);
  }
  tlog(LVL, "matching %i lines", lines.size());
  matchSubset(lines, objectSets);
  if(config_.mix_line_intersections) {
    tlog(LVL, "matching %i intersections", ints.size());
    matchSubset(ints, objectSets);
  } else {
    tlog(LVL, "matching %i Ts", ts.size());
    matchSubset(ts, objectSets);
    tlog(LVL, "matching %i Ls", ls.size());
    matchSubset(ls, objectSets);
  }
}

#define cmp(f) \
  if(left.f != right.f) return left.f < right.f;

// Sort the object sets so that updates are performed deterministically
// in the same order every time.
void Disambiguator::sortObjectSets(vector<ObjectSet>& objectSets) {
  for(auto& oset : objectSets) {
    std::sort(oset.begin(), oset.end(), [](WorldObject left, WorldObject right) {
      cmp(type);
      cmp(visionLine.start.x);
      cmp(visionLine.start.y);
      cmp(visionLine.end.x);
      cmp(visionLine.end.y);
      return false;
    });
  }
}

vector<vector<WorldObject>> Disambiguator::disambiguate(const vector<WorldObject>& queries, const set<WorldObjectType>& affinities) {
  tlog(LVL, "disambiguating %i queries", queries.size());
  thrown_out_ = 0;
  vector<vector<WorldObject>> objectSets = { vector<WorldObject>() };
  // I'm not really sure what this pruning is doing.
  auto mqueries = pruneDuplicateLines(queries);
  mqueries = pruneDuplicateIntersections(mqueries);
  bool used[NUM_WORLD_OBJS];
  memset(used, false, NUM_WORLD_OBJS);
  for(auto query : mqueries) {
    // Center circle and known lines/intersections are unique
    // This pushes back all these unique objects into all object sets
    if(query.isUnique() || query.isBall()) {
      used[query.type] = true;
      for(auto& objects : objectSets) {
        auto o = query;
        o.utype = query.type;
        objects.push_back(o);
      }
      continue;
    }
    // Skip known/unknown lines and known T and L intersections
    // Why are we skipping unknown lines? Does this matching happen somewhere else?
    if(query.isLine() || query.isT() || query.isL()) continue;
    // Generate candidates for unknown T/L intersections, and goal post lines 
    auto candidates = findCandidates(query);
    // Sort candidates by cost (this is computed in ObjectCandidate constructor)
    sortCandidates(query, candidates);
    selectCandidates(query, candidates, objectSets, used, affinities);
  }
  // This function actually does the matching between ambiguous queries and known world objects 
  matchObjects(mqueries, objectSets);
  sortObjectSets(objectSets);
  return objectSets;
}
