
#pragma once

#include <memory/TextLogger.h>
#include <vision/ObjectDetector.h>
#include <vision/structures/GoalPoint.h>
#include <vision/structures/FieldLine.h>
#include <vision/structures/CornerPoint.h>
#include <vision/enums/Colors.h>
#include <vision/BlobDetector.h>
#include <vision/LineDetector.h>
#include <vision/ColorSegmenter.h>
#include <common/ColorConversion.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vision/HoughDetector.h>
#include <vision/FieldEdgeDetector.h>
#include <vision/structures/GoalPostCandidate.h>
#include <vision/structures/GoalCandidate.h>
#include <vision/structures/HorizonLine.h>
#include <Eigen/Eigen>

class Classifier;

/// @ingroup vision
class GoalDetector : public ObjectDetector {
  public:
    GoalDetector(DETECTOR_DECLARE_ARGS, ColorSegmenter& color_segmenter, BlobDetector& blob_detector, LineDetector& line_detector, HoughDetector& hough_detector, FieldEdgeDetector& field_edge_detector);
    ~GoalDetector();

    // White goal detection!!
    void detectGoalPosts();


    void setHorizon(const HorizonLine& horizon) { horizon_ = horizon; }

    void detectWhiteGoal();     // Vision2017 new


    int postCount;    // Moved this here so we can compute statistics  
    GoalPostCandidate posts[2]; 



    void init(TextLogger* tl);



    float estimateGoalDistance(FieldLine * goal);
    float estimateGoalDistanceByKinematics(FieldLine* goal);
    float estimateGoalDistanceByHeight(float height);
    float estimateGoalDistanceByWidth(float width);
    float estimateGoalDistanceByPosts(Position left, Position right);


    const vector<GoalPostCandidate> getGoalPostCandidates() {return goalPostCandidates; }


    void setGoalObject(int goalIndex, float distance, float bearing, float elevation, int centerX, int centerY, float confidence, int lineIndex);

  
  private:

    ColorSegmenter& color_segmenter_;
    BlobDetector& blob_detector_;
    LineDetector& line_detector_;
    HoughDetector& hough_detector_;
    FieldEdgeDetector& field_edge_detector_;

    HorizonLine horizon_;

    unsigned char color;

    int totalValidGoals;


    float filterRobotLimbs(GoalPostCandidate *b);

    int hstep;
    int vstep;

    vector<GoalPostCandidate> goalPostCandidates;


    // Post candidate formation functions and sanity "pre-checks"
    void formGoalPostCandidates(const vector<Blob> &vertLines);
    int expandPost(const Blob *leftEdge, const Blob *rightEdge);
    float calculatePostWhitePercentage(int xi, int xf, int yi, int yf);
    float calculateGreenPercentBelow(int xi, int xf, int yi, int yf);


    void filterPosts();


    bool findGoalBox(int &line1, int &line2);
    float getOnGreenPercentage(GoalPostCandidate *b);


    float distToClosestLine(GoalPostCandidate *post);

    void formGoalFromPosts();

    // This sets world objects, etc. for localization
    void formWhiteGoal();


    static int totalPosts;
    std::unique_ptr<Classifier> classifier_;


    Coordinates transformPointInv(int x, int y);

    Eigen::Matrix3f H;
    Eigen::Matrix3f Hinv;


    float calculatePostWhitePercentage(LineSegment leftEdge, LineSegment rightEdge, int xstep, int ystep);



    float calculateEdgeBetweenStrength(LineSegment &line1, LineSegment &line2, cv::Mat &mat);

};

bool sortLineSegmentByXF(LineSegment line1, LineSegment line2);





