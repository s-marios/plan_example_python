from typing import Any
import numpy as np

import cv2
from sensor_msgs_py.point_cloud2 import read_points

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from sensor_msgs.msg import PointCloud2, Image

AREA_THRESHOLD = 2000
BG_INIT_FRAME_COUNT = 20
STABLE_FRAME_COUNT = 20
AREA_DIFF_RATIO = 0.03 #i.e. 3%

class ImageProcessor:

    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        self.count = 0
        self.br = CvBridge()
        self.depth_bg = cv2.createBackgroundSubtractorMOG2(500, 16)
        self.objectfound = False
        self.bg_init_frame_count = 0

        # Pointer to pointcloud processing function
        self.process_pc = self.bg_initialize

        self.node.create_subscription(
                PointCloud2, 
                "/camera/depth_registered/points", 
                self.callback, 10)

    def callback(self, msg: PointCloud2):
        self.process_pc(msg)

    ## this does not exist, just a function pointer to the appropriate state
    #def process_pc(self, msg: PointCloud2):
    #    pass

    def log_ndarray(self, a: np.ndarray, name = "array"):
        self.logger.info(f"{name}.shape is: {a.shape}, dtype: {a.dtype}, flags: {a.flags}")

    def process_depth_buffer(depth: np.ndarray) -> np.ndarray:
        dres = depth.copy()
        dres[ dres < .40] = 0
        dres[ dres > .60] = 0
        dres *= 256
        #todo: better scaling?? go full blast perhaps?
        return dres


    def get_buffers(msg: PointCloud2) -> (np.ndarray, np.ndarray, np.ndarray):
        ndpc = read_points(msg, reshape_organized_cloud=True)

        ndpc_rgb = ndpc['rgb'].reshape(ndpc.shape[1], ndpc.shape[0]).view((np.uint8, 4))
        depth = ndpc['z'].reshape(ndpc.shape[1], ndpc.shape[0])
        return ndpc_rgb, ImageProcessor.process_depth_buffer(depth), ndpc

    # returns the biggest contour and its area (if above threshold)
    def eval_contours(contours) -> tuple[int, Any] | None:
        def sortval(a):
            return a[0]

        if contours:
            sizes = [(cv2.contourArea(c), c) for c in contours]
            sizes.sort(reverse=True, key=sortval)
            print(f"contour area: {sizes[0][0]}")

            # if above threshold, return the biggest contour found
            if sizes[0][0] > AREA_THRESHOLD:
                return sizes[0]


    def bg_initialize(self, msg: PointCloud2):
        (ndpc_rgb, ndpc_depth, ndpc) = ImageProcessor.get_buffers(msg)

        # just update the depth background
        self.depth_bg.apply(ndpc_depth)

        self.bg_init_frame_count += 1
        if self.bg_init_frame_count > BG_INIT_FRAME_COUNT:
            self.process_pc = self.detect_object
            print(f"bg_initialize: move to detect object")

    def detect_object(self, msg: PointCloud2):
        #obtain image and depth buffers
        (ndpc_rgb, ndpc_depth, ndpc) = ImageProcessor.get_buffers(msg)

        # update background and get mask
        depth_mask = self.depth_bg.apply(ndpc_depth)

        #erode mask
        kernel = np.ones((5, 5), np.uint8)
        depth_mask = cv2.erode(depth_mask, kernel, 1)

        #find contours
        contours, h = cv2.findContours(depth_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #eval contours
        res = ImageProcessor.eval_contours(contours)

        # if there was a contour with area above threshold..
        if res:
            self.last_contour = res   
            self.process_pc = self.is_object_stable
            self.stable_frame_count = 0
            print(f"detect_object: object found")

    def is_object_stable(self, msg: PointCloud2):
        #obtain image and depth buffers
        ndpc_rgb, ndpc_depth, ndpc = ImageProcessor.get_buffers(msg)

        # an object appeared recently, DO NOT UPDATE background, get mask
        depth_mask = self.depth_bg.apply(ndpc_depth, learningRate=0)

        #erode mask
        kernel = np.ones((5, 5), np.uint8)
        depth_mask = cv2.erode(depth_mask, kernel, 1)

        #find contours
        contours, h = cv2.findContours(depth_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #eval contours
        res = ImageProcessor.eval_contours(contours)

        if res is None:
            #no object was found, go back to detecting objects
            print(f"moving back to self.detect_object")
            self.process_pc = self.detect_object
            return

        # an object was detected, calcualte area diff percentage
        (area, contour) = res
        diff = abs(self.last_contour[0] - area) / float(self.last_contour[0])
        print(f"self.stable_frame_count, diff: {self.stable_frame_count}, {diff}")

        # since an object was found, update the previously recorded contour
        self.last_contour = res

        # is the area coverage more than 3% different?
        if diff > AREA_DIFF_RATIO:
            # object unstable, do nothing
            self.stable_frame_count = 0
            print(f"reseting frame count")
            return

        # object stable, increase frame count
        self.stable_frame_count += 1
        if self.stable_frame_count > STABLE_FRAME_COUNT:
            self.compute_object_bb(res, ndpc)
            self.process_pc = self.await_removal

    def await_removal(self, msg: PointCloud2):
        print(f"do nothing until object is removed")

    def compute_object_bb(
            self,
            contour_tuple: tuple[int, Any],
            ndpc: np.ndarray):
        
        print(f"compute_object_bb")
        #0. get the other two nd.buffers from the message
        depth = ndpc['z'].reshape(ndpc.shape[1], ndpc.shape[0])
        width = ndpc['x'].reshape(ndpc.shape[1], ndpc.shape[0])
        height = ndpc['y'].reshape(ndpc.shape[1], ndpc.shape[0])
        rgb = ndpc['rgb'].reshape(ndpc.shape[1], ndpc.shape[0]).view((np.uint8, 4))

        #1. get a mask from the contour
        mask = np.zeros(depth.shape, np.uint8)
        cv2.drawContours(mask,[contour_tuple[1]], 0, 255, -1)

        self.log_ndarray(mask, "mask")
        pixelpoints = np.nonzero(mask)

        #2. apply the contour to all three buffers
        masked_depth = depth[pixelpoints]
        masked_width = width[pixelpoints]
        masked_height = height[pixelpoints]


        #3. pretty pictures
        rgb_cont = cv2.drawContours(rgb.copy(), [contour_tuple[1]], 0, (0, 0, 255), 3)
        cv2.imwrite("/tmp/rgb_cont.bmp", rgb_cont)
        cv2.imwrite("/tmp/mask.bmp", mask)
        cv2.imwrite("/tmp/depth.bmp", depth.copy() * 256)
        depth_proc = ImageProcessor.process_depth_buffer(depth.copy())
        cv2.imwrite("/tmp/depth_proc.bmp", depth_proc)
        cv2.imwrite("/tmp/back.bmp", self.depth_bg.getBackgroundImage())

        def compute_stats(buff: np.ndarray) -> (float, float, float, float):
            buff_min = buff.min()
            buff_max = buff.max()
            return (buff.mean(), buff_min, buff_max, buff_max - buff_min)

        #4. for each of those, compute min,max,avg,spread
        stats = [compute_stats(buff) for buff in [masked_depth, masked_width, masked_height]]
        for (stat, dimension) in zip(stats, ["depth", "width", "height"]):
            print(f"dimension: {dimension}, avg/min/max/diff: {stat}")


def main():
    rclpy.init()
    logger = get_logger("img_proc")
    node = rclpy.create_node("img_proc")

    imgproc = ImageProcessor(node, logger)
    rclpy.spin(imgproc.node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
