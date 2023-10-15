import rclpy
from rclpy.node import Node as rclpy_Node

from common.srv import GetImage
from std_msgs.msg import String
from common.msg import ImageSource

import time

class BaseImageSource(rclpy_Node):
    def __init__(self, node_name="image_source_emulator"):
        super().__init__(node_name)

        self.image_service = self.create_service(
            GetImage,
            node_name+"_service",
            self.request_callback,
        )

        self.request_subscriber = self.create_subscription(
            String,
            f"villa/{node_name}/request",
            self.rq_topic_callback,
            10)
        
        self.image_publisher = self.create_publisher(ImageSource,
                                                  f"villa/{node_name}/response", 10)
    
    def request_callback(self, request, response):
        response.image = self.load_image(request.args)
        return response

    def rq_topic_callback(self, msg):
        # t0 = time.time()
        img_msg = self.load_image(msg.data)
        # print(f"Image loading time: {time.time()-t0}")
        # t0 = time.time()
        self.image_publisher.publish(img_msg)
        # print(f"Publish time: {time.time()-t0}")
    
    def load_image(self, args):
        raise NotImplementedError

