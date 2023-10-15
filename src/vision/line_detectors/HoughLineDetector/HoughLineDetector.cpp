#include<HoughLineDetector.hpp>


float compute_angle(Vec4i line)
{
    float deltaX = (float)((line[2] - line[0]));
    float deltaY = (float)((line[3] - line[1]));
    float angle;

    if(abs(deltaX) <= 0.01 * sqrt((pow(deltaX, 2) + pow(deltaY, 2)))) {
        angle = 1000; // set to a large value (1000)
    }
    else {
        angle = abs(atan2(deltaY, deltaX) + M_PI);
        if (angle >= M_PI)
            angle -= M_PI;
    }

    return angle;
}


FieldLine::FieldLine(Vec4i line)
: line_(line), numLinesInGroup_(1)
{
    angle_ = compute_angle(line);
}

float FieldLine::distanceBetweenTwoPoints(Vec2i p1, Vec2i p2) {
    return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
}

Vec2i FieldLine::getMidpoint(Vec4i line) {
    Vec2i midpoint;

    midpoint[0] = (float)((line[0] + line[2]) / 2);
    midpoint[1] = (float)((line[1] + line[3]) / 2);

    return midpoint;
}

float FieldLine::distanceToLineGroup(Vec4i line) {
    int lineGroupInd;
    int newLineInd;
    float leastDist = 1000;
    float currDist;
    int leastLineGroupInd;
    int leastNewLineInd;

    for(lineGroupInd = 0; lineGroupInd <= 3; lineGroupInd += 2) {
        for(newLineInd = 0; newLineInd <= 3; newLineInd += 2) {
            Vec2i lineGroupEndpoint = Vec2i(line_[lineGroupInd], line_[lineGroupInd + 1]);
            Vec2i newLineEndpoint = Vec2i(line[newLineInd], line[newLineInd + 1]);
            if(abs(angle_) >= 1000)
                currDist = abs(lineGroupEndpoint[0] - newLineEndpoint[0]);
            else
            {
                Vec4i crossLine(lineGroupEndpoint[0], lineGroupEndpoint[1], newLineEndpoint[0], newLineEndpoint[1]);
                float crossLineAngle = compute_angle(crossLine);
                currDist = distanceBetweenTwoPoints(lineGroupEndpoint, newLineEndpoint)*abs(sin(crossLineAngle - angle_));
            }
            if(currDist < leastDist) {
                leastDist = currDist;
                leastLineGroupInd = lineGroupInd;
                leastNewLineInd = newLineInd;
            }
        }
    }

    Vec2i pointLineGroup = Vec2i(line_[leastLineGroupInd], line_[leastLineGroupInd + 1]);
    Vec2i pointNewLine = Vec2i(line[leastNewLineInd], line[leastNewLineInd + 1]);
    
    return leastDist;
}

// TODO: add distance thresholds to reduce based on distance/line angle
bool FieldLine::partOfLine(Vec4i newLine, float distThresh = 5, float angleThresh = 0.2) {

    float newLineAngle = compute_angle(newLine);

    if(abs(newLineAngle) >= 1000) {
        // newLine is vertical, check is line group's line is vertical too
        if(abs(angle_) < 1000) {
            return false;
        }
    }
    else {
        if(min(abs(newLineAngle - angle_), (float)M_PI - abs(newLineAngle - angle_)) > angleThresh) {
            return false;
        }
    }

    float distanceBetweenLines = distanceToLineGroup(newLine);

    if(distanceBetweenLines <= distThresh) {
        return true;
    }

    return false; 
}

void FieldLine::updateEndpoints(Vec4i line) {

    if(abs(angle_) >= 1000)
    {
        line_[0] = line_[2] = (line_[0] + line[0] + line_[2] + line[2]) / 4.0;
        float new_y1 = min(line_[1], min(line_[3], min(line[1], line[3])));
        float new_y2 = max(line_[1], max(line_[3], max(line[1], line[3])));
        line_[1] = new_y1;
        line_[2] = new_y2;
    }

    float min_x = min(line_[0], min(line_[2], min(line[0], line[2])));
    float max_x = max(line_[0], max(line_[2], max(line[0], line[2])));

    float y_new_proj = line[1] + (line_[0] - line[0])* tan(angle_);
    y_new_proj = (y_new_proj + line_[1])/2.0;

    line_[1] = y_new_proj + (min_x - line_[0]) * tan(angle_);
    line_[3] = y_new_proj + (max_x - line_[0]) * tan(angle_);
    line_[0] = min_x;
    line_[2] = max_x;
}

void FieldLine::updateAngle() {
    angle_ = compute_angle(line_);
}

void FieldLine::addLineToGroup(Vec4i newLine) {
    updateEndpoints(newLine);
    updateAngle();
    numLinesInGroup_++;
}

Vec4i FieldLine::getLine() {
    return line_;
}

size_t FieldLine::getNumLines() {
    return numLinesInGroup_;
}


HoughLineDetector::HoughLineDetector(string& name_, Json::Value& config) : LineDetector(name_, config)
{
    configure(config);
}


void HoughLineDetector::configure(Json::Value config)
{
    LineDetector::configure(config);
    ReadJSONAttribute(config, color_segmenter_name, String);

    ReadJSONAttributeIfExists(config, input_subsample_major, Int);
    ReadJSONAttributeIfExists(config, input_subsample_minor, Int);

    ReadJSONAttribute(config, distance_threshold, Float);
    ReadJSONAttribute(config, angle_threshold, Float);
    
    ReadJSONAttribute(config, hough_rho, Float);
    ReadJSONAttribute(config, hough_theta, Float);
    ReadJSONAttribute(config, hough_threshold, Int);
    ReadJSONAttribute(config, hough_min_line_length, Int);
    ReadJSONAttribute(config, hough_max_line_gap, Int);

    if(config.isMember("detect_roi"))
    {
        auto config_roi = config["detect_roi"];
        if(config_roi.size() != 4)
        {
            RCLCPP_ERROR(this->get_logger(), "HoughLineDetector '%s': Wrong number of values for ROI. Expected 4, got %d", stage_name.c_str(), config_roi.size());
            exit(1);
        }
        int i = 0;
        for(auto v : config_roi)
        {
            detect_roi[i++] = v.asFloat();
        }
    }
    else
    {
        detect_roi = Vec4f(0.0, 0.0, 1.0, 1.0);
    }
}


vector<Vec4i> HoughLineDetector::detect_region(shared_ptr<ColorSegmenter> color_segmenter, Vec4f roi, Vec2i& roi_size)
{
    Mat full_image_bgr, bgr_image, bgr_image_resized, src;
    Mat dst, cdst, cdstP;

    struct CopyConfig copy_config;
    copy_config.subsample_major = input_subsample_major;
    copy_config.subsample_minor = input_subsample_minor;
    copy_config.input_channels = 3;
    Mat segmented_image = color_segmenter->get_image_matrix();

    Vec4i roi_pixels(roi.mul(
        Vec4f(segmented_image.cols, segmented_image.rows, segmented_image.cols, segmented_image.rows)
        ) - Vec4f(0, 0, 1, 1)
    );

    roi_size[0] = segmented_image.cols; roi_size[1] = segmented_image.rows;

    for(int i=0; i<4; i++)
        copy_config.rect[i] = roi_pixels[i];
    bgr_image = color_segmenter->copy_to_mat(copy_config);

    cvtColor(bgr_image, src, COLOR_BGR2GRAY);

    // Gaussian Blur
    Mat blurred_img;
    GaussianBlur(src, blurred_img, Size(5, 5), 0);
    // Edge detection
    Canny(blurred_img, dst, 50, 200, 3);

    // Probabilistic Hough Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(
        dst,
        linesP,
        hough_rho,
        hough_theta,
        hough_threshold,
        hough_min_line_length,
        hough_max_line_gap
    ); // runs the actual detection

    RCLCPP_DEBUG(this->get_logger(), "Total %d lines", (int)linesP.size());

    // scale and shift the lines to account for cropping and subsampling
    float scale = copy_config.get_scale_factor();
    for(auto it = linesP.begin(); it != linesP.end(); it++)
    {
        (*it)[0] = (*it)[0] * scale + roi_pixels[0];
        (*it)[2] = (*it)[2] * scale + roi_pixels[0];
        (*it)[1] = (*it)[1] * scale + roi_pixels[1];
        (*it)[3] = (*it)[3] * scale + roi_pixels[1];
    }

    return linesP;
}


vector<msg_ImageLine> HoughLineDetector::detect(unordered_map<string, shared_ptr<VisionStage>>& stages)
{
    auto color_segmenter = dynamic_pointer_cast<ColorSegmenter>(stages[color_segmenter_name]);
    
    // Run detection in configured ROI
    Vec2i roi_size;
    auto hough_lines = detect_region(color_segmenter, detect_roi, roi_size);

    // Group lines
    vector<FieldLine> lineGroups;
    for( size_t i = 0; i < hough_lines.size(); i++ )
    {
        Vec4i l = hough_lines[i];

        if(lineGroups.size() == 0) {
            lineGroups.push_back(FieldLine(l));
        }
        else {
            bool added = false;
            for(size_t j = 0; j < lineGroups.size() && !added; j++) {
                if(lineGroups[j].partOfLine(l, distance_threshold, angle_threshold)) {
                    lineGroups[j].addLineToGroup(l);
                    added = true;
                }
            }

            if(!added) {
                lineGroups.push_back(FieldLine(l));
            }
        }
    }

    vector<msg_ImageLine> detected_lines;
    // Collect detected lines in message format
    for(size_t k = 0; k < lineGroups.size(); k++) {
        Vec4i lineToDraw = lineGroups[k].getLine();
        auto line_ = msg_ImageLine();
        line_.x1 = ((float)lineToDraw[0])/roi_size[0];
        line_.y1 = ((float)lineToDraw[1])/roi_size[1];
        line_.x2 = ((float)lineToDraw[2])/roi_size[0];
        line_.y2 = ((float)lineToDraw[3])/roi_size[1];

        line_.confidence = 1.0;

        detected_lines.push_back(line_);
    }
    
    return detected_lines;
}
