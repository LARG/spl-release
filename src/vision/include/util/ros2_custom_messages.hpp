#include <ros2_base.hpp>

#include "common/msg/image_object.hpp"
#include "common/msg/image_object_list.hpp"

#include "common/msg/image_line.hpp"
#include "common/msg/image_line_list.hpp"

#include "common/msg/image_source.hpp"

typedef std_msgs::msg::String msg_InvokeVision;

typedef common::msg::ImageObject msg_ImageObject;
typedef common::msg::ImageObjectList msg_DetectedObjects;

typedef common::msg::ImageLine msg_ImageLine;
typedef common::msg::ImageLineList msg_DetectedLines;

typedef common::msg::ImageSource msg_ImageSource;
