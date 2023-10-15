/**
 * @author Felix Weiglhofer
 */

#pragma once

#include "merge_channels.hpp"
#include "fourier_transform.h"
#include "rectangular_smooth.h"
#include "whistle_classifier.h"
#include "alsarecorder.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include <fftw3.h>

class WhistleDetector :  public rclcpp::Node {

public:
    WhistleDetector(AudioStream &, size_t minWhistleLength, 
            float whistleThreshold);
    
    void process();
    bool debug();

    void start();

    void stop();

    bool isRunning() { return _running; }

    bool hasCandidate() { return _whistleClassifier.hasCandidate(); }
    float currPeak() { return _whistleClassifier.currPeak(); }

    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    
    AudioStream &_audioProvider;
    MergeChannels<fftwf_malloc, fftwf_free> _mergeChannels;
    FourierTransform _fourierTransform;
    RectangularSmooth _rectSmooth;
    WhistleClassifier _whistleClassifier;

    bool _running;

};

// vim: set ts=4 sw=4 sts=4 expandtab:
