#include "localization/line_matching.hpp"
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

using std::vector;
using Eigen::Vector3d;

double line_matching::sin_2_theta(Line l1, Line l2) {
  Vector2d end_l1 = l1.b - l1.a;
  Vector2d end_l2 = l2.b - l2.a;

  double cross_product = end_l1[0] * end_l2[1] - end_l1[1] * end_l2[0];

  double sin_theta = cross_product/(end_l1.norm() * end_l2.norm());

  return sin_theta * sin_theta;
}

/**
 * Calculates the distance between a point and its projection on a given line
 * segement.
 */
double line_matching::distance_between_line_point(Line l, Vector2d p) {
  Vector2d vec_ab = l.b - l.a;
  Vector2d vec_bp = p - l.b;
  Vector2d vec_ap = p - l.a;

  if (vec_ab.dot(vec_bp) > 0)
    return vec_bp.norm();
  else if (vec_ab.dot(vec_ap) < 0)
    return vec_ap.norm();
  else
    return abs(vec_ab[0] * vec_ap[1] - vec_ap[0] * vec_ab[1]) /
      vec_ab.norm();
}

/**
 * Similarity metric for two line segments.
 */
double line_matching::distance_between_lines(Line l1, Line l2) {
  return std::min(line_matching::distance_between_line_point(l2, l1.a) +
      line_matching::distance_between_line_point(l2, l1.b),
      line_matching::distance_between_line_point(l1, l2.a) +
      line_matching::distance_between_line_point(l1, l2.b));
    //std::min((l1.b-l1.a).norm(), (l2.b-l2.a).norm()) *
    //sqr(line_matching::sin_2_theta(l1, l2));
}

/**
 * Returns the angle of a line segment. The output is in (-PI/2, PI/2), so
 * swapping the endpoints doesn't change the result.
 */
float line_matching::compute_angle(const Line line) {
  return atan((line.b[1] - line.a[1]) / (line.b[0] - line.a[0]));
}

int line_matching::line_match(Line l) {
  int best_line = 0;
  double min_dist = line_matching::distance_between_lines(
      l, line_matching::field_lines[0]);

  for (int i = 1; i < NUM_LINES; i++) {
    double dist = line_matching::distance_between_lines(
        l, line_matching::field_lines[i]);
    if (dist < min_dist) {
      min_dist = dist;
      best_line = i;
    }
  }
  return best_line;
}

vector<int> line_matching::line_match(vector<Line> lines) {
  vector<int> matches(lines.size());
  for (size_t i = 0; i < lines.size(); i++) {
    matches[i] = line_matching::line_match(lines[i]);
  }
  return matches;
}

/**
 * Project point onto line segement.
 */
Vector2d line_matching::project_point_onto_line(Vector2d p, Line l) {
  Vector2d vec_ab = l.b - l.a;
  Vector2d vec_bp = p - l.b;
  Vector2d vec_ap = p - l.a;

  if (vec_ab.dot(vec_bp) > 0) {
    return l.b;
  } else if (vec_ab.dot(vec_ap) < 0) {
    return l.a;
  } else {
    return l.a + vec_ab * (vec_ap.dot(vec_ab) / vec_ab.dot(vec_ab));
  }
}

Eigen::Vector3d line_matching::get_error_correction(
    std::vector<int> &matches,
    std::vector<Line> &lines,
    std::vector<bool> &line_matches_accepted,
    float angle_threshold,  // radians
    float distance_threshold) {
  Vector2d xy_err {0.0, 0.0};
  float yaw_err = 0.0;

  for (int i = 0; i < lines.size(); i++) {
    Line actual = line_matching::field_lines[matches[i]];
    Line predicted = lines[i];

    float angle_actual = compute_angle(actual);
    float angle_pred = compute_angle(predicted);
    float angle_diff = angle_pred - angle_actual;
    // Turning by 180 degrees is still matching the line
    if (angle_diff > M_PI/2) {
      angle_diff -= M_PI;
    } else if (angle_diff < -M_PI/2) {
      angle_diff += M_PI;
    }
    // Project points of predicted line onto actual line
    Vector2d pred_proj_a = line_matching::project_point_onto_line(
        predicted.a, actual);
    Vector2d pred_proj_b = line_matching::project_point_onto_line(
        predicted.b, actual);
    auto xy_diff = (pred_proj_a - predicted.a) + (pred_proj_b - predicted.b);

    if (abs(angle_diff) < angle_threshold) {
      if (xy_diff.norm() < distance_threshold) {
        line_matches_accepted[i] = true;
        yaw_err += angle_diff;
        xy_err += xy_diff;
        /*RCLCPP_INFO(rclcpp::get_logger("line_matching"),
          "\n(%f, %f)\t(%f, %f)\n%s\tAngle diff %f, Distance diff %f",
          predicted.a[0], predicted.a[1],
          predicted.b[0], predicted.b[1],
          line_matching::field_line_names[matches[i]].c_str(),
          angle_diff, xy_diff.norm());*/
      } else {
        /*RCLCPP_INFO(rclcpp::get_logger("line_matching"),
          "\n(%f, %f)\t(%f, %f)\n%s\tREJECTED\tDistance diff %f too large",
          predicted.a[0], predicted.a[1],
          predicted.b[0], predicted.b[1],
          line_matching::field_line_names[matches[i]].c_str(),
          xy_diff.norm());*/
      }
    } else {
      /*RCLCPP_INFO(rclcpp::get_logger("line_matching"),
        "\n(%f, %f)\t(%f, %f)\n%s\tREJECTED\tAngle diff %f too large",
        predicted.a[0], predicted.a[1],
        predicted.b[0], predicted.b[1],
        line_matching::field_line_names[matches[i]].c_str(),
        angle_diff);*/
    }
  }

  // TODO: explore other options (weighted average)
  float x_err = xy_err[0] / lines.size();
  float y_err = xy_err[1] / lines.size();
  yaw_err /= lines.size();

  return Vector3d{x_err, y_err, yaw_err};
}
