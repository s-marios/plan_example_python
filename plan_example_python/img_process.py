import numpy as np

import cv2
from sensor_msgs_py.point_cloud2 import read_points

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from sensor_msgs.msg import PointCloud2, Image


class ImageProccessor:

    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        self.count = 0
        self.br = CvBridge()
        self.back = cv2.createBackgroundSubtractorKNN()

        self.node.create_subscription(
                PointCloud2, 
                "/camera/depth_registered/points", 
                self.sub_callback, 10)


    def sub_callback(self, msg: PointCloud2):
        self.count = (self.count + 1) % 10

        self.logger.info("I received a message!") 
        self.logger.info(f"msg info: \n  isbigendian? {msg.is_bigendian}, point_step: {msg.point_step}")
        self.logger.info(f"row_step? {msg.row_step}, h/w: {msg.height}/{msg.width}")
        for field in msg.fields:
            self.logger.info(f"field info: {field.name}, {field.offset}, {field.datatype}, {field.count}")
        self.logger.info(f"data len: {len(msg.data)}, is_dense? {msg.is_dense}")

        ndpc = read_points(msg, reshape_organized_cloud=True)
        self.logger.info(f"ndpc.shape is: {ndpc.shape}, dtype: {ndpc.dtype}, flags are: {ndpc.flags}")

        #TODO: why do we have to reshape the array? row/column-major order
        # someone is lying to me
        ndpc_rgb = ndpc['rgb'].reshape(ndpc.shape[1], ndpc.shape[0]).view((np.uint8, 4))

        self.logger.info(f"ndpc_rgb.shape is: {ndpc_rgb.shape}, dtype: {ndpc_rgb.dtype}")

        mask = self.back.apply(ndpc_rgb)

        kernel = np.ones((5, 5), np.uint8)
        mask_er = cv2.erode(mask, kernel, 1)

        if self.count == 0:
            cv2.imwrite("/tmp/depth.bmp", ndpc_rgb)
            cv2.imwrite("/tmp/mask.bmp", mask)
            cv2.imwrite("/tmp/mask_er.bmp", mask_er)



def main():
    rclpy.init()
    logger = get_logger("img_proc")
    node = rclpy.create_node("img_proc")

    imgproc = ImageProccessor(node, logger)
    rclpy.spin(imgproc.node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
