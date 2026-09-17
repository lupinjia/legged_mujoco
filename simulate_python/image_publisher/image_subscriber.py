import time
import numpy as np
import cv2 as cv

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from idl._DepthImage_ import DepthImage_

def DepthImageHandler(msg: DepthImage_):
    
    # print width, height and first 5 data points
    # print(f"Received Depth Image: width={msg.width}, height={msg.height}, data[0:5]={msg.data[0:5]}")
    
    # show the passed depth image in a window
    height = msg.height
    width = msg.width
    depth_image = np.array(msg.data, dtype=np.float32).reshape((height, width))
    pixels = (255 * depth_image).astype(np.uint8) # already normalized
    cv.imshow("Depth Image", pixels)
    cv.waitKey(1)

if __name__ == "__main__":
    ChannelFactoryInitialize(1, "lo")
    sub = ChannelSubscriber("rt/depthimage", DepthImage_)
    sub.Init(DepthImageHandler, 10)
    
    while True:
        time.sleep(10.0)