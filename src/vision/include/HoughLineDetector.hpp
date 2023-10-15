#include <LineDetector.hpp>
#include <ColorSegmenter.hpp>


class FieldLine
{
public:
    Vec4i line_;
    float angle_;
    size_t numLinesInGroup_;
    void updateEndpoints(Vec4i line);
    void updateAngle();
    float distanceToLineGroup(Vec4i line);
    float distanceBetweenTwoPoints(Vec2i p1, Vec2i p2);
    Vec2i getMidpoint(Vec4i line);


    FieldLine(Vec4i line);
    void addLineToGroup(Vec4i newLine);
    bool partOfLine(Vec4i, float, float);
    Vec4i getLine();
    size_t getNumLines();
};


class HoughLineDetector: public LineDetector
{
    string color_segmenter_name;
    int input_subsample_major = 1;
    int input_subsample_minor = 1;
    float distance_threshold;
    float angle_threshold;
    float hough_rho;
    float hough_theta;
    int hough_threshold;
    int hough_min_line_length;
    int hough_max_line_gap;
    Vec4f detect_roi;
    public:
    HoughLineDetector(string&, Json::Value&);
    vector<Vec4i> detect_region(shared_ptr<ColorSegmenter>, Vec4f, Vec2i&);
    vector<msg_ImageLine> detect(unordered_map<string, shared_ptr<VisionStage>>&);
    void configure(Json::Value);
};
