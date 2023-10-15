#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "localization/line_matching.hpp"

using geometry_msgs::msg::Point;
using visualization_msgs::msg::Marker;

class FieldLinesVisualizationNode : public rclcpp::Node {
 public:
    FieldLinesVisualizationNode() : Node("field_lines_visualization") {
      publisher_ = this->create_publisher<Marker>("field_lines", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
          std::bind(&FieldLinesVisualizationNode::timer_callback, this));
    }

 private:
    void timer_callback() {
      Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "field_lines";
      marker.id = 0;
      marker.type = Marker::LINE_LIST;

      marker.scale.x = 50;

      marker.color.r = 1.0;
      marker.color.b = 1.0;
      marker.color.g = 1.0;
      marker.color.a = 1.0;
      for (int i = 0; i < line_matching::NUM_LINES; i++) {
        Point p1;
        p1.x = line_matching::field_lines[i].a[0];
        p1.y = line_matching::field_lines[i].a[1];
        p1.z = 0.0;
        Point p2;
        p2.x = line_matching::field_lines[i].b[0];
        p2.y = line_matching::field_lines[i].b[1];
        p2.z = 0.0;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }
      const int num_circle_points = 100;
      const int center_circle_radius = line_matching::CENTER_CIRCLE_DIAMETER/2;
      for (int i = 0; i < num_circle_points; i++) {
        Point p1;
        p1.x = center_circle_radius * cos(2*M_PI*i/num_circle_points);
        p1.y = center_circle_radius * sin(2*M_PI*i/num_circle_points);
        p1.z = 0.0;
        Point p2;
        p2.x = center_circle_radius * cos(2*M_PI*(i+1)/num_circle_points);
        p2.y = center_circle_radius * sin(2*M_PI*(i+1)/num_circle_points);
        p2.z = 0.0;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }
      publisher_->publish(marker);
    }

    rclcpp::Publisher<Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FieldLinesVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
