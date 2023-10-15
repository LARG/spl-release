import os
import glob
import sys
import time
from typing import Iterable

import numpy as np

import rclpy
from rclpy.node import Node as rclpy_Node

from common.msg import ImageSource as msg_ImageSource

from image_source_emulators.base_node import BaseImageSource


BYTES_TO_IGNORE = 36


class LogReplay(BaseImageSource):
    def __init__(self,
                 base_log_dir :str,
                 filename_prefix :str,
                 image_dims :Iterable[int],
                 format :str ="YUYV",
                 node_name:str ="log_replay",
                ):
        self.node_name = node_name
        super().__init__(self.node_name)

        self.base_log_dir = base_log_dir
        self.filename_prefix = filename_prefix
        self.format = format

        if self.format != "YUYV":
            raise NotImplementedError

        self.image_filenames = glob.glob(os.path.join(base_log_dir, filename_prefix+"*"))
        self.image_filenames.sort()

        print("File list:")
        for s in self.image_filenames:
            print(s)

        self.image_width, self.image_height = image_dims
        
        if len(self.image_filenames) == 0:
            raise ValueError("No image files found")
        
        self.i =0

    def load_image(self, args):
        fname = self.image_filenames[self.i]
        raw_bytes = None
        
        # t0 = time.time()
        with open(fname,'rb') as im_fh:
            raw_bytes = np.frombuffer(im_fh.read(), dtype=np.uint8)
        # print(f"File read time: {time.time() - t0}")

        # t0 = time.time()
        image_msg = msg_ImageSource()
        image_msg.image.frombytes(raw_bytes[BYTES_TO_IGNORE:])
        # print(f"Msg assignment time: {time.time() - t0}")

        image_msg.image_width = self.image_width
        image_msg.image_height = self.image_height
        image_msg.source = self.node_name
        image_msg.color_space = self.format

        self.i = (self.i+1)%len(self.image_filenames)

        return image_msg


def main():
    print('Starting log_replay.')

    base_log_dir = sys.argv[1]
    filename_prefix = sys.argv[2]
    image_dims = (int(sys.argv[3]), int(sys.argv[4]))
    
    rclpy.init()
    node = LogReplay(base_log_dir,
                     filename_prefix,
                     image_dims
                    )
    executer = rclpy.executors.SingleThreadedExecutor()
    executer.add_node(node)
    executer.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()