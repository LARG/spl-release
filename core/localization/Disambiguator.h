#pragma once

#include <memory/MemoryCache.h>
#include <common/WorldObject.h>
#include <common/Concepts.h>
#include <localization/LocalizationBase.h>
#include <math/PairwiseCostMergeUtility.h>
#include <math/Geometry.h>
#include <functional>
#include <set>

class TextLogger;

template<typename ObjectT, typename CostT>
class PairwiseCostMergeUtility;

struct ObjectCandidate {
  WorldObject obj;
  float cost;
  TextLogger* tlogger_;
  ObjectCandidate(Point2D sloc, float sorient, WorldObject obj, const WorldObject& query, TextLogger* tlogger);
};

/** This class is responsible for matching ambiguous (i.e. UNKNOWN) world objects to known world objects. */

class Disambiguator : public LocalizationBase {
  public:
    Disambiguator(BASE_DECLARATIONS);
    void updateSelf(Point2D sloc, float sorient) { sloc_ = sloc; sorient_ = sorient; }
    std::vector<ObjectSet> disambiguate(const std::vector<WorldObject>& queries, const std::set<WorldObjectType>& affinities);
    int thrownOut() { return thrown_out_; }
  private:
    static float distance(const WorldObject* left, const WorldObject* right);
    static WorldObject* mergeIntersections(const WorldObject* left, const WorldObject* right);
    static WorldObject* mergeLines(const WorldObject* left, const WorldObject* right);
    std::vector<WorldObject> pruneDuplicates(const std::vector<WorldObject>& objects, std::function<WorldObject*(const WorldObject*,const WorldObject*)> mergeFunction, std::function<bool(const WorldObject&)> predicate);
    std::vector<WorldObject> pruneDuplicateLines(const std::vector<WorldObject>& objects);
    std::vector<WorldObject> pruneDuplicateIntersections(const std::vector<WorldObject>& objects);
    std::vector<WorldObject> pruneNetDetections(const vector<WorldObject>& queries);
    std::vector<WorldObject> pruneCirclePieces(const vector<WorldObject>& queries);
    void sortObjectSets(std::vector<ObjectSet>& objectSets);

    std::vector<int> findCandidateTypes(const WorldObject& query);
    std::vector<ObjectCandidate> findCandidates(const WorldObject& query);
    void sortCandidates(const WorldObject& query, std::vector<ObjectCandidate>& candidates);
    void selectCandidates(const WorldObject& query, vector<ObjectCandidate>& candidates, vector<ObjectSet>& objectSets, bool* used, const std::set<WorldObjectType>& affinities);
    void addCandidate(const WorldObject& query, ObjectCandidate& cand, vector<ObjectSet>& objectSets, bool* used);
    Point2D sloc_;
    float sorient_;

    void matchSubset(std::vector<WorldObject> queries, std::vector<ObjectSet>& objectSets);
    void matchObjects(const std::vector<WorldObject>& queries, std::vector<ObjectSet>& objectSets);
    int thrown_out_;
    using MergeUtility = PairwiseCostMergeUtility<WorldObject,float>;
    std::unique_ptr<MergeUtility> merge_utility_;
    static Disambiguator* instance_;
};
