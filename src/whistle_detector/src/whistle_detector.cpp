#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "whistle_detector/whistle_detector.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;


WhistleDetector::WhistleDetector(
  AudioStream & audioProvider,
  size_t minWhistleLength,
  float whistleThreshold)
: Node("whistle_detector"), _audioProvider(audioProvider),
  _mergeChannels(
    _audioProvider.getBuffer(),
    _audioProvider.getBufferSize(),
    _audioProvider.getChannelNum()),
  _fourierTransform(
    _mergeChannels.output(),
    _mergeChannels.outSize(),
    _audioProvider.getSampleRate()),
  _rectSmooth(
    3,
    _fourierTransform.output(),
    _fourierTransform.outSize(),
    _fourierTransform.freqSpacing()),
  _whistleClassifier(
    _rectSmooth.output(),
    _rectSmooth.outSize(),
    whistleThreshold,
    minWhistleLength,
    _mergeChannels.outSize(),
    _rectSmooth.freqSpacing(),
    _rectSmooth.freqOffset()),
  _running(false)
{
  publisher_ = this->create_publisher<std_msgs::msg::Bool>("/whistle", 10);
  timer_ = this->create_wall_timer(
    10ms, std::bind(&WhistleDetector::process, this));
}


bool WhistleDetector::debug() {
    assert(_running);
    _audioProvider.fetch();
    _mergeChannels.execute();
    _fourierTransform.execute();
    _rectSmooth.execute();
    _whistleClassifier.execute();

    return _whistleClassifier.whistleDetected();
}


void WhistleDetector::process()
{
  assert(_running);
  _audioProvider.fetch();
  _mergeChannels.execute();
  _fourierTransform.execute();
  _rectSmooth.execute();
  _whistleClassifier.execute();


  // Scan for whistle.
  if (_whistleClassifier.whistleDetected()) {
     std_msgs::msg::Bool msg;
     msg.data = true;
     publisher_->publish(msg);
     RCLCPP_INFO(this->get_logger(), "Published: %d", msg.data);
  }
  else{
    std_msgs::msg::Bool msg;
    msg.data = false; 
     // Publish the message
    publisher_->publish(msg); 

  }


}

void WhistleDetector::start()
{
  assert(!_running);
  _audioProvider.start();
  _running = true;
}

void WhistleDetector::stop()
{
  assert(_running);
  _audioProvider.stop();
  _whistleClassifier.reset();
  _running = false;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  AlsaRecorder a(AlsaRecorder::V5_SETTINGS);
  auto w = std::make_shared<WhistleDetector>(a, AlsaRecorder::V5_SETTINGS.sampleRate * 0.4, 2000);
  w->start();
  rclcpp::spin(w);
  rclcpp::shutdown();
  return 0;
}
