#pragma once

#include <memory>
#include <memory/TextLogger.h>
#include <vision/ObjectDetector.h>
#include <vision/enums/Colors.h>
#include <vision/ColorTableMethods.h>
#include <vision/ROIDetector.h>
#include <vision/structures/GoalPostCandidate.h>
#include <vision/structures/GoalPostBaseCandidate.h>
#include <common/ColorConversion.h>
#include <opencv2/core/core.hpp>

/// @ingroup vision
class GoalDetector : public ObjectDetector {
 public:
  GoalDetector(DETECTOR_DECLARE_ARGS);
  ~GoalDetector() = default;

  void init(TextLogger *tl) {
    textlogger = tl;
  };

  void detectGoalPosts(const unsigned char *color_table, const std::vector<cv::Point> &field_edge_points);

  inline vector<pair<int, int>> const & getFieldBoundaryColor() const {
    return field_boundary_color_;
  };

  inline vector<GoalPostBaseCandidate> const & goalPostBaseCandidates() const {
    return goal_post_base_candidates_;
  };

  inline vector<GoalPostCandidate> const & getGoalPostCandidates() const {
    return goal_post_candidates_;
  };

  inline vector<cv::Point> const & getHull() const {
    return hull_;
  };

 private:
  TextLogger* textlogger;

  //Timer goalTimer;

  /* Instance variables only used for debugging */
  // Contains field boundary points - only used for debugging
  vector<pair<int, int>> field_boundary_color_;
  // Contains points of the convex hull encapsulating the field - only used for debugging
  vector<cv::Point> hull_;
  vector<GoalPostBaseCandidate> goal_post_base_candidates_;

  vector<GoalPostCandidate> goal_post_candidates_;
  vector<GoalPostCandidate> valid_goal_post_candidates_;

  void clear_fields();
  std::vector<cv::Point> constructFieldHull(const unsigned char *color_table, const std::vector<cv::Point> &field_edge_points);
  std::vector<GoalPostBaseCandidate> constructBaseCandidates(const unsigned char *color_table, const std::vector<cv::Point> &hull);
  void filterBaseCandidates(std::vector<GoalPostBaseCandidate> &candidates);
  void setGoalObject(int goalIndex, float distance, float bearing, float elevation, int centerX, int centerY, float confidence, int lineIndex);
  float estimateGoalDistanceByPosts(Position left, Position right);

  inline const unsigned char* getImage() const {
    if(camera_ == Camera::TOP)
      return vblocks_.image->getImgTop();
    return vblocks_.image->getImgBottom();	
  }

  inline Color xy2color(const unsigned char *color_table, int x, int y) const {
    return ColorTableMethods::xy2color(getImage(),
        color_table, x, y, iparams_.width);
  }

  inline void xy2yuv(int xPos, int yPos, int &y, int &u, int &v) const {
    ColorTableMethods::xy2yuv(getImage(), xPos, yPos, iparams_.width, y, u, v);
  }

};
