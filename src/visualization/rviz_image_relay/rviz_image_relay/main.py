import rclpy
from rclpy.node import Node as rclpy_Node

from sensor_msgs.msg import Image
from common.msg import ImageSource, ImageObjectList, ImageLineList
from cv_bridge import CvBridge

import cv2
import numpy as np

class ImageRelay(rclpy_Node):
    def __init__(self):
        super().__init__("rviz_image_relay")

        self.image_subscriber = self.create_subscription(
            ImageSource,
            'villa/vision/published_images',
            self.image_callback,
            10)

        self.image_sources_topics = {
            "nao_camera_top": None,
            "nao_camera_bottom": None,
            "static_image_top_1": None,
            "static_image_bottom_1": None,
        }

        self.detector_sources = {
            "ball_detector_top" : "nao_camera_top",
            "line_detector_top": "nao_camera_top",
            "ball_detector_bottom" : "nao_camera_bottom"
        }

        self.detected_objects_sources = self.image_sources_topics.copy()
        self.detected_lines_sources = self.image_sources_topics.copy()

        for im_topic in self.image_sources_topics:
            self.image_sources_topics[im_topic] = self.create_publisher(
                Image, f"villa/rviz_images/{im_topic}", 10)
            self.detected_objects_sources[im_topic] = []
            self.detected_lines_sources[im_topic] = []

        self.detected_objects_subscriber = self.create_subscription(
            ImageObjectList,
            'villa/vision/detected_objects',
            self.objects_callback,
            10)

        self.detected_lines_subscriber = self.create_subscription(
            ImageLineList,
            'villa/vision/detected_lines',
            self.lines_callback,
            10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        """Processes image and publishes annotated image."""
        img_source = msg.source
        # img_bytes = np.array(msg.image, dtype=np.uint8)
        # prepend_bytes = np.zeros((36, ), dtype=np.uint8)
        # raw_img_bytes = np.concatenate((prepend_bytes, img_bytes), axis=0).tobytes()
        # file_name = f"../{img_source}.yuv"
        raw_image = np.array(msg.image, dtype=np.uint8).reshape(
            (msg.image_height, msg.image_width, 2))
        annot_image = cv2.cvtColor(raw_image, cv2.COLOR_YUV2RGB_YUYV)

        def norm_coord_to_point(x, y):
            return tuple((np.array((x, y))*np.array([msg.image_width,
                                                     msg.image_height])).astype(int))

        # Draw detected objects
        for obj in self.detected_objects_sources[img_source]:
            top_left = norm_coord_to_point(obj.center_x - 0.5*obj.size_x,
                                           obj.center_y - 0.5*obj.size_y)
            bottom_right = norm_coord_to_point(obj.center_x + 0.5*obj.size_x,
                                               obj.center_y + 0.5*obj.size_y)
            annot_image = cv2.rectangle(annot_image, top_left, bottom_right, (255, 255, 0), 2)

        # Draw detected lines
        for line in self.detected_lines_sources[img_source]:
            pt1 = norm_coord_to_point(line.x1, line.y1)
            pt2 = norm_coord_to_point(line.x2, line.y2)
            annot_image = cv2.line(annot_image, pt1, pt2, (0, 255, 255), 2)

        image_message = self.bridge.cv2_to_imgmsg(annot_image, encoding="rgb8")
        self.image_sources_topics[img_source].publish(image_message)

        # Clear detected objects
        self.detected_objects_sources[img_source] = []
        self.detected_lines_sources[img_source] = []

    def objects_callback(self, msg):
        for obj in msg.objects:
            im = self.detector_sources[obj.source]
            self.detected_objects_sources[im].append(obj)

    def lines_callback(self, msg):
        for line in msg.lines:
            im = self.detector_sources[line.source]
            self.detected_lines_sources[im].append(line)


def main():
    print('Starting rviz_image_relay.')

    rclpy.init()
    node = ImageRelay()
    executer = rclpy.executors.SingleThreadedExecutor()
    executer.add_node(node)
    executer.spin()

    rclpy.shutdown()



if __name__ == '__main__':
    main()
