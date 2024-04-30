import cv2

cam_index = [0,2,4,6]

for x in range(4):
    cam = cv2.VideoCapture(cam_index[x])
    ret, image = cam.read()
    filepath = '/home/avmi-cam-sys/cam_images/testimage' + str(x+1) + '.png'
    cv2.imwrite(filepath, image)
    cam.release()
