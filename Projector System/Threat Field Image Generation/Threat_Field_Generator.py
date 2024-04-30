## This script is meant to be run on the ground control station to publish the images to the headless Raspberry PIs via NFS drive
import math
import GCS_Functions as GCS

# THREAT FIELD PARAMETERS
img_height = 800
img_width = 1280
mu_x = 250
mu_y = 250
sigma_x = 150
sigma_y = 100
saturation = 0.8
value = 1.0
theta = math.pi/12
scale_factor = 0.8
invert = False


# def display_image():
#     GCS.generate_image_with_rot(img_height, img_width, mu_x, mu_y, sigma_x, sigma_y, saturation, value, theta, scale_factor, invert)
#     Images = GCS.split_image('test_dist.png')


# while (True):

#     while mu_x > 200:
#         mu_x -= 25
#         display_image()

#     while mu_x < 600:
#         mu_x += 25
#         display_image()


# Generate Gaussian threat field
GCS.generate_image_with_rot(800, 1280, 250, 400, 200, 400, saturation, value, theta, scale_factor, invert)   # Good distribution
# GCS.generate_image(800, 1280, 100, 100, 60, 180, 0.8, 1.0)      # Temp distribution for 1 projector
# GCS.generate_image_with_rot(img_height, img_width, mu_x, mu_y, sigma_x, sigma_y, saturation, value, theta, scale_factor, invert)      # Temp distribution for 1 projector

# Split image and return 4 PNGs
Images = GCS.split_image('test_dist.png')



