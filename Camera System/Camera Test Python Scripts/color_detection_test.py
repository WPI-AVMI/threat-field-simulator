import time
import libcamera
import cv2
import colorsys
import numpy as np
from picamera2 import Picamera2, Preview

picam = Picamera2()

config = picam.create_preview_configuration(main={"size": (800, 600)})
config["transform"] = libcamera.Transform(hflip=1, vflip=1)
picam.configure(config)

#picam.start_preview(Preview.QTGL)

picam.start()

while(True):
    
    time.sleep(2)
    picam.capture_file("/home/avmi-lab-04/Pictures/test_image.jpg")
    
    image = cv2.imread('/home/avmi-lab-04/Pictures/test_image.jpg")
    blue = image[400][300][0]
    green = image[400][300][1]
    red = image[400][300][2]
    
    r = red/255
    g = green/255
    b = blue/255
    
    hsv_color = colorsys.rgb_to_hsv(r, g, b)
    
    #print(hsv_color)
    
    hue = hsv_color[0]
    #print(hue)
    
    hue_scaled = hue*360
    print(hue_scaled)
