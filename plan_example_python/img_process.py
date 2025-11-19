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
                #"/camera/depth/points", 
                self.sub_callback, 10)
        #self.node.create_subscription(
        #        Image,
        #        "/camera/color/image_raw",
        #        self.sub_color, 10)


    def sub_callback(self, msg: PointCloud2):
        self.logger.info("I received a message!") 
        self.logger.info(f"msg info: \n  isbigendian? {msg.is_bigendian}, point_step: {msg.point_step}")
        self.logger.info(f"row_step? {msg.row_step}, h/w: {msg.height}/{msg.width}")
        for field in msg.fields:
            self.logger.info(f"field info: {field.name}, {field.offset}, {field.datatype}, {field.count}")
        self.logger.info(f"data len: {len(msg.data)}, is_dense? {msg.is_dense}")
        ndpc = read_points(msg, reshape_organized_cloud=True)
        self.logger.info(f"ndpc.shape is: {ndpc.shape}, dtype: {ndpc.dtype}")
        self.logger.info("read a point cloud!")

        ndpc_rgb = np.asfortranarray(ndpc['rgb']).reshape(720,1280).view((np.uint8, 4))

        self.logger.info(f"ndpc_rgb.shape is: {ndpc_rgb.shape}, dtype: {ndpc_rgb.dtype}")

        if self.count == 0:
            #out = cv2.imdecode(ndpc_rgb, cv2.IMREAD_COLOR)
            out = cv2.cvtColor(ndpc_rgb, cv2.COLOR_RGBA2BGR)
            self.logger.info(f"out.shape is: {out.shape}, dtype: {out.dtype}")
            cv2.imwrite("/tmp/depth.bmp", out)


    def sub_color(self, msg: Image):
        self.count = (self.count + 1) % 30 
        self.logger.info("I received an Image!") 
        self.logger.info(f"info w/h/e: {msg.width}, {msg.height}, {msg.encoding}")
        if self.count == 0:
            cvimg = self.br.imgmsg_to_cv2(msg, "rgb8")
            self.logger.info(f"  cvimg shape: {cvimg.shape} dtype: {cvimg.dtype}")
            mask = self.back.apply(cvimg)
            cv2.imwrite("test.jpg", cvimg)
            cv2.imwrite("mask.jpg", mask)
        #raw = np.frombuffer(msg.data, dtype=np.uint8)#.reshape(1280, 720, 3)
        #cv2_read = cv2.imdecode(raw, cv2.IMREAD_ANYCOLOR)#.reshape(1280, 720, 3)



def main():
    rclpy.init()
    logger = get_logger("img_proc")
    node = rclpy.create_node("img_proc")

    imgproc = ImageProccessor(node, logger)
    rclpy.spin(imgproc.node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
