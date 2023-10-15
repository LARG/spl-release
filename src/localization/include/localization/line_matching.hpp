#ifndef _LINE_MATCHING
#define _LINE_MATCHING
#define sqr(x) (x)*(x)

#include <limits.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <cmath>
#include "localization/CameraMatrix.h"
#include "common/msg/image_line.hpp"

using Eigen::Vector2d;
using common::msg::ImageLine;

struct Line {
    Vector2d a;
    Vector2d b;

    Line(Vector2d x, Vector2d y): a(x), b(y) {}
    Line(float x1, float y1, float x2, float y2):
      a(Vector2d{x1, y1}), b(Vector2d{x2, y2}) {}
    Line(): a(Vector2d{0, 0}), b(Vector2d{0, 0}) {}
};


namespace line_matching {
    const float BALL_RADIUS = 51;

    const float FIELD_X = 9000;                 // Rules: A (9000) UT: 8000
    const float FIELD_Y = 6000;                 // Rules: B (6000)
    const float LINE_WIDTH = 50;                // Rules: C (50)
    const float PENALTY_MARK_SIZE = 100;        // Rules: D (100) UT: 106
    const float GOAL_AREA_X =  600;             // Rules: E (600)
    const float GOAL_AREA_Y = 2200;             // Rules: F (2200)
    const float PENALTY_AREA_X =  1650;         // Rules: G (1650)
    const float PENALTY_AREA_Y =  4000;         // Rules: H (4000)
    const float PENALTY_MARK_DISTANCE = 1300;   // Rules: I (1300)
    const float CENTER_CIRCLE_DIAMETER = 1500;  // Rules: J (1500)
    const float BORDER_STRIP_WIDTH = 700;       // Rules: K (700) UT: 300

    const float GOAL_Y = 1500;          // Rules: Goal inner width (Fig. 2)
    const float GOAL_POST_WIDTH = 100;  // Rules: Post width (Fig. 2)
    const float GOAL_X = 500;           // Rules: Distance from endline to back
                                        //   of goal (Fig. 2)
    const float GOAL_HEIGHT = 800;      // Rules: Height of the goal, to the
                                        //   bottom of the top bar (Fig. 3)

    const float GRASS_Y = FIELD_Y + 2 * BORDER_STRIP_WIDTH;
    const float GRASS_X = FIELD_X + 2 * BORDER_STRIP_WIDTH;
    const float HALF_FIELD_Y = FIELD_Y/2.0;
    const float HALF_FIELD_X = FIELD_X/2.0;
    const float HALF_GRASS_Y = GRASS_Y/2.0;
    const float HALF_GRASS_X = GRASS_X/2.0;

    enum FIELD_LINES {
        CENTER_LINE = 0,
        PENALTY_FRONT_POS,
        PENALTY_LEFT_POS,
        PENALTY_RIGHT_POS,
        GOAL_FRONT_POS,
        GOAL_LEFT_POS,
        GOAL_RIGHT_POS,
        PENALTY_FRONT_NEG,
        PENALTY_LEFT_NEG,
        PENALTY_RIGHT_NEG,
        GOAL_FRONT_NEG,
        GOAL_LEFT_NEG,
        GOAL_RIGHT_NEG,
        END_LINE_POS,
        END_LINE_NEG,
        SIDE_LINE_LEFT,
        SIDE_LINE_RIGHT,
        NUM_LINES
    };

    const std::string field_line_names[NUM_LINES] = {
        "CENTER_LINE",
        "PENALTY_FRONT_POS",
        "PENALTY_LEFT_POS",
        "PENALTY_RIGHT_POS",
        "GOAL_FRONT_POS",
        "GOAL_LEFT_POS",
        "GOAL_RIGHT_POS",
        "PENALTY_FRONT_NEG",
        "PENALTY_LEFT_NEG",
        "PENALTY_RIGHT_NEG",
        "GOAL_FRONT_NEG",
        "GOAL_LEFT_NEG",
        "GOAL_RIGHT_NEG",
        "END_LINE_POS",
        "END_LINE_NEG",
        "SIDE_LINE_LEFT",
        "SIDE_LINE_RIGHT"
    };

    const Line field_lines[NUM_LINES] = {
        // CENTER_LINE
        Line(0, HALF_FIELD_Y, 0, -HALF_FIELD_Y),
        // PENALTY_FRONT_POS
        Line(HALF_FIELD_X-PENALTY_AREA_X, PENALTY_AREA_Y/2,
             HALF_FIELD_X-PENALTY_AREA_X, -PENALTY_AREA_Y/2),
        // PENALTY_LEFT_POS
        Line(HALF_FIELD_X-PENALTY_AREA_X, PENALTY_AREA_Y/2,
             HALF_FIELD_X,                PENALTY_AREA_Y/2),
        // PENALTY_RIGHT_POS
        Line(HALF_FIELD_X-PENALTY_AREA_X, -PENALTY_AREA_Y/2,
             HALF_FIELD_X,                -PENALTY_AREA_Y/2),
        // GOAL_FRONT_POS
        Line(HALF_FIELD_X-GOAL_AREA_X, GOAL_AREA_Y/2,
             HALF_FIELD_X-GOAL_AREA_X, -GOAL_AREA_Y/2),
        // GOAL_LEFT_POS
        Line(HALF_FIELD_X-GOAL_AREA_X, GOAL_AREA_Y/2,
             HALF_FIELD_X,             GOAL_AREA_Y/2),
        // GOAL_RIGHT_POS
        Line(HALF_FIELD_X-GOAL_AREA_X, -GOAL_AREA_Y/2,
             HALF_FIELD_X,             -GOAL_AREA_Y/2),
        // PENALTY_FRONT_NEG
        Line(-HALF_FIELD_X+PENALTY_AREA_X, PENALTY_AREA_Y/2,
             -HALF_FIELD_X+PENALTY_AREA_X, -PENALTY_AREA_Y/2),
        // PENALTY_LEFT_NEG
        Line(-HALF_FIELD_X+PENALTY_AREA_X, PENALTY_AREA_Y/2,
             -HALF_FIELD_X,                PENALTY_AREA_Y/2),
        // PENALTY_RIGHT_NEG
        Line(-HALF_FIELD_X+PENALTY_AREA_X, -PENALTY_AREA_Y/2,
             -HALF_FIELD_X,                -PENALTY_AREA_Y/2),
        // GOAL_FRONT_NEG
        Line(-HALF_FIELD_X+GOAL_AREA_X, GOAL_AREA_Y/2,
             -HALF_FIELD_X+GOAL_AREA_X, -GOAL_AREA_Y/2),
        // GOAL_LEFT_NEG
        Line(-HALF_FIELD_X+GOAL_AREA_X, GOAL_AREA_Y/2,
             -HALF_FIELD_X,             GOAL_AREA_Y/2),
        // GOAL_RIGHT_NEG
        Line(-HALF_FIELD_X+GOAL_AREA_X, -GOAL_AREA_Y/2,
             -HALF_FIELD_X,             -GOAL_AREA_Y/2),
        // END_LINE_POS
        Line(HALF_FIELD_X, HALF_FIELD_Y, HALF_FIELD_X, -HALF_FIELD_Y),
        // END_LINE_NEG
        Line(-HALF_FIELD_X, HALF_FIELD_Y, -HALF_FIELD_X, -HALF_FIELD_Y),
        // SIDE_LINE_LEFT
        Line(-HALF_FIELD_X, HALF_FIELD_Y, HALF_FIELD_X, HALF_FIELD_Y),
        // SIDE_LINE_RIGHT
        Line(-HALF_FIELD_X, -HALF_FIELD_Y, HALF_FIELD_X, -HALF_FIELD_Y)
    };

    double sin_2_theta(Line, Line);
    double distance_between_line_point(Line, Vector2d);
    double distance_between_lines(Line, Line);
    int line_match(Line);
    std::vector<int> line_match(std::vector<Line>);
    Eigen::Vector3d get_error_correction(
        std::vector<int>&,
        std::vector<Line>&,
        std::vector<bool>&,
        float angle_threshold = 0.4,
        float distance_threshold = 500);
    Vector2d project_point_onto_line(Vector2d p, Line l);
    float compute_angle(const Line);
}  // namespace line_matching

#endif
