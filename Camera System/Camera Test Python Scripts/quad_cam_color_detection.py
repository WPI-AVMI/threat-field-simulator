import cv2
import colorsys
import numpy as np

# ID of each of the 4 cameras: /dev/videoX
cam_index = [0,2,4,6]

color_vals = [0,0,0,0]

for n in range(4):
    
    # Take image
    cam = cv2.VideoCapture(cam_index[n])
    #cam.set(3,320)
    #cam.set(4,240)
    ret, image = cam.read()
    filepath = '/home/avmi-cam-sys/cam_images/testimage' + str(n+1) + '.png'
    cv2.imwrite(filepath, image)
    cam.release()
    
    # Get color from image
    
    # Sample region parameters
    center_x = 320
    center_y = 240
    half_range_x = 2
    half_range_y = 2
    num_pixels = (half_range_x*2 + 1) * (half_range_y*2 + 1)

    # Initialize RGB values
    blue = 0
    green = 0
    red = 0
    
    # Compute RGB values from center 5x5 pixel grid
    for x in range(center_x - half_range_x, center_x + half_range_x):
        for y in range(center_y - half_range_y, center_y + half_range_y):
            blue += image[x][y][0] / num_pixels
            green += image[x][y][1] / num_pixels
            red += image[x][y][2] / num_pixels
    
    # Normalize RGB values
    r = red/255
    g = green/255
    b = blue/255
    
    # Convert from RGB to HSV
    hsv_color = colorsys.rgb_to_hsv(r, g, b)
    
    # Get normalized HSV value
    hue = hsv_color[0]
    color_vals[n] = hue
    
    # Debug print for fun
    print(hue*360)


print(color_vals)


