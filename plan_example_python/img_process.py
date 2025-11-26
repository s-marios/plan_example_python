import numpy as np

import cv2
from sensor_msgs_py.point_cloud2 import read_points

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from sensor_msgs.msg import PointCloud2, Image

def sortval(a):
        return a[0]

class ImageProccessor:

    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        self.count = 0
        self.br = CvBridge()
        self.back = cv2.createBackgroundSubtractorKNN()
        self.bd = cv2.createBackgroundSubtractorMOG2(500, 16)
        self.objectfound = False

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
        #ndpc_x = ndpc['x'].reshape(ndpc.shape[1], ndpc.shape[0])
        #ndpc_x = cv2.imdecode(ndpc_x, cv2.IMREAD_UNCHANGED)
        ndpc_x = ndpc['z'].reshape(ndpc.shape[1], ndpc.shape[0])
        self.logger.info(f"ndpc_x.max is: {ndpc_x.max()}, min: {ndpc_x.min()}")
        ndpc_x[ ndpc_x < .40 ] = 0
        ndpc_x[ ndpc_x > .60 ] = 0
        ndpc_x *= 256 
        #ndpc_x *= 1024 
        #ndpc_x *= 4096 

        self.logger.info(f"ndpc_rgb.shape is: {ndpc_rgb.shape}, dtype: {ndpc_rgb.dtype}")
        self.logger.info(f"ndpc_x.shape is: {ndpc_x.shape}, dtype: {ndpc_x.dtype}")

        if self.count % 5 == 0:

            mask = self.back.apply(ndpc_rgb)
            depth_mask = self.bd.apply(ndpc_x)

            kernel = np.ones((5, 5), np.uint8)
            mask_er = cv2.erode(mask, kernel, 1)
            depth_mask = cv2.erode(depth_mask, kernel, 1)

            #_, threshold_mask = cv2.threshold(depth_mask, 1, 255, cv2.THRESH_BINARY)
            #contours, h = cv2.findContours(threshold_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            #cimg = cv2.drawContours(depth_mask, contours, -1, (0, 255, 0))


            gray = depth_mask
            #gray = cv2.cvtColor(depth_mask, cv2.COLOR_RGB2GRAY)

            contours, h = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            print(f"gray dtype: {gray.dtype}, shape: {gray.shape}, contour count: {len(contours)}")

            
            ## visualize the contours
            rgbgray = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGBA)
            #img_c = cv2.drawContours(rgbgray, contours, -1, (0, 255, 0, 0), 3)


            if contours:
                sizes = [(cv2.contourArea(c), c) for c in contours]
                sizes.sort(reverse=True, key=sortval)
                img_c = cv2.drawContours(rgbgray, [sizes[0][1]], -1, (0, 0, 255), -1)
                print(f"contour area: {sizes[0][0]}")

            if self.count == 0:
                cv2.imwrite("/tmp/depth.bmp", ndpc_rgb)
                cv2.imwrite("/tmp/mask.bmp", mask)
                cv2.imwrite("/tmp/mask_er.bmp", mask_er)
                cv2.imwrite("/tmp/depth_x.bmp", ndpc_x)
                cv2.imwrite("/tmp/mask_depth.bmp", depth_mask)
                if contours:
                    cv2.imwrite("/tmp/img_c.bmp", img_c)



def main():
    rclpy.init()
    logger = get_logger("img_proc")
    node = rclpy.create_node("img_proc")

    imgproc = ImageProccessor(node, logger)
    rclpy.spin(imgproc.node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
