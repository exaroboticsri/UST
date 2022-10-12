#https://pysource.com
import pyrealsense2 as rs
import numpy as np
import threading

import os
import sys

import time
import queue
import threading

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
sys.path.extend([INCLUDE_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH

from Include.Commons import PySignal


class ThreadRealsense(threading.Thread):
    keepAlive: bool = True

    def __init__(self):
        super(ThreadRealsense, self).__init__(target=self.run)
        self.setDaemon(True)
        self.sig_get_frame = PySignal(tuple)
        self.cam = CRealsenseCamera()
        
        self.prevTime = 0.
        self.fps = 0.
        
    def run(self) -> None:
        while True:
            color, depth = self.cam.get_frame_stream()
            
            currTime = time.time()
            self.fps = 1./(currTime - self.prevTime)
            
            self.prevTime = currTime
            
            param = self.cam.param
            self.sig_get_frame.emit((color, depth, param, self.fps))
        
    def stop(self):
        self.keepAlive = False
        
class CRealsenseCamera:
    def __init__(self):
        # Configure depth and color streams
        print("Loading Intel Realsense Camera")
        self.pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 15)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)

        # Start streaming
        self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        self.color_intrinsics = color_profile.get_intrinsics()
        print('Loading Intel Realsense Camera: get_intrinsics')
        fx = self.color_intrinsics.fx
        fy = self.color_intrinsics.fy
        ppx = self.color_intrinsics.ppx
        ppy = self.color_intrinsics.ppy
        width = 1280
        hight = 720
        
        self.param = (fx, fy, ppx, ppy, width, hight)
        # print(self.param)
        # cam_params=[self.color_intrinsics.fx,self.color_intrinsics.fy,self.color_intrinsics.ppx,self.color_intrinsics.ppy]
        # w, h = self.color_intrinsics.width, self.color_intrinsics.height
        self.prevTime = 0.
        self.fps = 0.
        print("Loading Intel Realsense Camera: OK")

    def get_frame_stream(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            # If there is no frame, probably camera not connected, return False
            print("Error, impossible to get the frame, make sure that the Intel Realsense camera is correctly connected")
            return None, None
        
        # Apply filter to fill the Holes in the depth image
        # spatial = rs.spatial_filter()
        # spatial.set_option(rs.option.holes_fill, 3)
        # filtered_depth = spatial.process(depth_frame)

        # hole_filling = rs.hole_filling_filter()
        # filled_depth = hole_filling.process(filtered_depth)

        
        # Create colormap to show the depth of the Objects
        # colorizer = rs.colorizer()
        # depth_colormap = np.asanyarray(colorizer.colorize(filled_depth).get_data())

        currTime = time.time()
        self.fps = 1./(currTime - self.prevTime)
        
        self.prevTime = currTime
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # print(self.fps)
        return color_image, depth_image
    
    def release(self):
        self.pipeline.stop()


