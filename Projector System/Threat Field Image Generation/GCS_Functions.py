from PIL import Image
import colorsys
import math
import numpy as np


class Point: 
    def __init__(self, x, y): 
        self.x = x
        self.y = y


def generate_image(height, width, mu_x, mu_y, sigma_x, sigma_y, saturation, value):

    filename = "test_dist.png"

    pixelArray = [[0 for i in range(width)] for j in range(height)]
    for x in range(height):
        for y in range(width):
            pixelArray[x][y] = hue2rgb(xyFunction(x, y, mu_x, mu_y, sigma_x, sigma_y), saturation, value)

    img = Image.fromarray(np.array(pixelArray), mode="RGB")
    img.save(filename)


def generate_image_with_rot(height, width, mu_x, mu_y, sigma_x, sigma_y, saturation, value, theta, scale_factor, invert):

    filename = "test_dist.png"

    pixelArray = [[0 for i in range(width)] for j in range(height)]
    for x in range(height):
        for y in range(width):
            newPoint = calc_rot_point(x, y, mu_x, mu_y, theta)
            pixelArray[x][y] = hue2rgb(xyFunction(newPoint.x, newPoint.y, mu_x, mu_y, sigma_x, sigma_y), saturation, value, scale_factor, invert)

    img = Image.fromarray(np.array(pixelArray), mode="RGB")
    img.save(filename)


def calc_rot_point(x,y, mu_x, mu_y, theta):

    # Translate mean to origin
    newX = x - mu_x
    newY = y - mu_y

    # Perform rotation
    newX = newX * math.cos(theta) - newY * math.sin(theta)
    newY = newX * math.sin(theta) + newY * math.cos(theta)

    # Translate mean back to original position
    newX = newX + mu_x
    newY = newY + mu_y

    return Point(newX, newY)


def xyFunction(x, y, mu_x, mu_y, sigma_x, sigma_y):
    A = 2 * math.pi * (sigma_x ** 2) * (sigma_y ** 2)
    B = (x - mu_x) ** 2
    C = 2 * (sigma_x ** 2)
    D = (y - mu_y) ** 2
    F = 2 * (sigma_y ** 2)
    return math.exp(-((B/C) + (D/F)))


def hue2rgb(hue, saturation, value, scale_factor, invert):
    if invert:
        hue = scale_factor - hue*scale_factor
    else:
        hue = hue*scale_factor
    return tuple(np.uint8(component * 255) for component in colorsys.hsv_to_rgb(hue, saturation, value))


def split_image(image_path):

    # Open the original image
    original_image = Image.open(image_path)

    # Get the dimensions of the original image
    width, height = original_image.size

    # Calculate the size of each cut image (1280x800)
    cut_width, cut_height = width/2, height/2

    #Define empty binary images
    binary_images = []

    img_num = 0

    # Create four cut images
    for row in range(2):
        for col in range(2):
            # Calculate the region to crop for each cut image
            left = col * cut_width
            upper = row * cut_height
            right = left + cut_width
            lower = upper + cut_height

            # Crop the region and save as a new PNG
            split_image = original_image.crop((left, upper, right, lower))

            # Flip images 1 and 2 vertically due to projector setup
            if row == 0:
                split_image = split_image.transpose(Image.ROTATE_180)

            img_num += 1
            filename = "/var/nfs/projector-pics/projector-" + str(img_num) + ".png"

            split_image.save(filename)
    
