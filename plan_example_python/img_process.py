from typing import Any
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

        self.process_pc = self.bg_initialize
        self.bg_init_frame_count = 0

        self.node.create_subscription(
                PointCloud2, 
                "/camera/depth_registered/points", 
                self.callback, 10)

    ## this does not exist, just a function pointer to the appropriate state
    #def process_pc(self, msg: PointCloud2):
    #    pass

    def get_buffers(msg: PointCloud2) -> (np.ndarray, np.ndarray, np.ndarray):
        ndpc = read_points(msg, reshape_organized_cloud=True)

        ndpc_rgb = ndpc['rgb'].reshape(ndpc.shape[1], ndpc.shape[0]).view((np.uint8, 4))
        ndpc_x = ndpc['z'].reshape(ndpc.shape[1], ndpc.shape[0])
        ndpc_x[ ndpc_x < .40 ] = 0
        ndpc_x[ ndpc_x > .60 ] = 0
        #todo: better scaling?? go full blast perhaps?
        ndpc_x *= 256 
        return ndpc_rgb, ndpc_x, ndpc

    # returns the biggest contour and its area (if above threshold)
    def eval_contours(contours) -> tuple[int, Any] | None:
        if contours:
            sizes = [(cv2.contourArea(c), c) for c in contours]
            sizes.sort(reverse=True, key=sortval)
            #img_c = cv2.drawContours(rgbgray, [sizes[0][1]], -1, (0, 0, 255), -1)
            print(f"contour area: {sizes[0][0]}")

            # if above threshold, return the biggest contour found
            if sizes[0][0] > 2000:
                return sizes[0]

    def compute_stats(buff: np.ndarray) -> (float, float, float, float):
        buff_min = buff.min()
        buff_max = buff.max()
        return (buff.mean(), buff_min, buff_max, buff_max - buff_min)


    def bg_initialize(self, msg: PointCloud2):

        (ndpc_rgb, ndpc_depth, ndpc) = ImageProccessor.get_buffers(msg)

        # just apply the masks
        mask = self.back.apply(ndpc_rgb)
        depth_mask = self.bd.apply(ndpc_depth)

        self.bg_init_frame_count += 1
        if self.bg_init_frame_count > 20:
            self.process_pc = self.detect_object
            print(f"bg_initialize: move to detect object")

    def detect_object(self, msg: PointCloud2):
        #obtain image and depth buffers
        (ndpc_rgb, ndpc_depth, ndpc) = ImageProccessor.get_buffers(msg)

        # update background and get mask
        depth_mask = self.bd.apply(ndpc_depth)

        #erode mask
        kernel = np.ones((5, 5), np.uint8)
        depth_mask = cv2.erode(depth_mask, kernel, 1)

        #find contours
        contours, h = cv2.findContours(depth_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #eval contours
        res = ImageProccessor.eval_contours(contours)
        if res:
            self.last_contour = res   
            self.process_pc = self.is_object_stable
            self.stable_frame_count = 0
            print(f"detect_object: object found")

    def is_object_stable(self, msg: PointCloud2):
        #obtain image and depth buffers
        ndpc_rgb, ndpc_depth, ndpc = ImageProccessor.get_buffers(msg)

        # an object appeared recently, DO NOT UPDATE background, get mask
        depth_mask = self.bd.apply(ndpc_depth, learningRate=0)

        #erode mask
        kernel = np.ones((5, 5), np.uint8)
        depth_mask = cv2.erode(depth_mask, kernel, 1)

        #find contours
        contours, h = cv2.findContours(depth_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #eval contours
        res = ImageProccessor.eval_contours(contours)
        if res:
            (area, contour) = res
            diff = abs(self.last_contour[0] - area) / float(self.last_contour[0])
            print(f"self.stable_frame_count, diff: {self.stable_frame_count}, {diff}")

            # is the area coverage less than 3% different?
            if diff < .03:
                # object stable, increase frame count
                self.stable_frame_count += 1
                if self.stable_frame_count > 20:
                    self.compute_object_bb(res, ndpc)
                    self.process_pc = self.await_removal
            else:
                # object unstable, do nothing
                self.stable_frame_count = 0
                print(f"reseting frame count")

            # an object was found, update the previously recorded contour
            self.last_contour = res

        else:
            #object was not found, go back to detecting objects
            print(f"moving back to self.detect_object")
            self.process_pc = self.detect_object

    def await_removal(self, msg: PointCloud2):
        print(f"do nothing until object is removed")

    def compute_object_bb(
            self,
            contour_tuple: tuple[int, Any],
            ndpc: np.ndarray):
        
        print(f"compute_object_bb")
        #0. get the other two nd.buffers from the message
        depth = ndpc['x'].reshape(ndpc.shape[1], ndpc.shape[0])
        width = ndpc['z'].reshape(ndpc.shape[1], ndpc.shape[0])
        height = ndpc['y'].reshape(ndpc.shape[1], ndpc.shape[0])

        #1. get a mask from the contour
        mask = np.zeros((depth.shape[1], depth.shape[0]), np.uint8)
        cv2.drawContours(mask,[contour_tuple[1]], 0, 255, -1)
        pixelpoints = np.nonzero(mask)

        #2. apply the contour to all three buffers
        masked_depth = depth[pixelpoints]
        masked_width = width[pixelpoints]
        masked_height = height[pixelpoints]

        #3. for each of those, compute min,max,avg,spread
        stats = [ImageProccessor.compute_stats(buff) for buff in [masked_depth, masked_width, masked_height]]
        for (stat, dimension) in zip(stats, ["depth", "width", "height"]):
            print(f"dimension: {dimension}, avg/min/max/diff: {stat}")


    def callback(self, msg: PointCloud2):
        self.process_pc(msg)

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
