# Original ideas from https://towardsdatascience.com/finding-driving-lane-line-live-with-opencv-f17c266f15db
# Additional ideas incorporated from 
# Original credit to abhinav
# Published 20 April 2018
# https://www.hackster.io/user8146544/road-lane-detection-with-raspberry-pi-a4711f
# Maze navigation using canny edge detection and houghlinesP transform
# v1.1

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import math

# setup variables
theta = 0
minLineLength = 5
maxLineGap = 10

# setup camera
camera = PiCamera()
camera.rotation = 180
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640,480))
# warmup camera
time.sleep(0.1)

# capture and work with the video

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    # apply gray scaling
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
    # apply blurring
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size,kernel_size),0)
    # edges  
    low_threshold = 50
    high_threshold = 150
    edged = cv2.Canny(blur_gray, low_threshold, high_threshold)
    
    # create a mask of the edges image using cv2.filpoly()
    mask = np.zeros_like(edged)
    ignore_mask_color = 255

    # define the Region of Interest (ROI) - source code sets as a trapezoid for roads
    imshape = image.shape
      
    vertices = np.array([[(0,imshape[0]),(450, 320), (500, 320),(imshape[1],imshape[0])]], dtype=np.int32)

    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_edges = cv2.bitwise_and(edged, mask)


    # define the Hough Transform parameters
    rho = 2 # distance resolution in pixels of the Hough grid
    theta2 = np.pi/180 # angular resolution in radians of the Hough grid
    threshold = 15     # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 40 #minimum number of pixels making up a line - original 40
    max_line_gap = 30    # maximum gap in pixels between connectable line segments - original 30

    # make a blank the same size as the original image to draw on
    # uncomment if rotating image 
    line_image = np.copy(image)*0 

    # run HoughLinesP on masked version of edge detected image
    # uncomment when using ROI 
    lines = cv2.HoughLinesP(masked_edges, rho, theta2, threshold, np.array([]),min_line_length, max_line_gap)
    if(lines !=None):
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)
                theta=theta+math.atan2((y2-y1),(x2-x1))    
                
            
    # draw the line on the original image 
    lines_edges = cv2.addWeighted(image, 0.8, line_image, 1, 0) #uncomment if rotating image
    #return lines_edges

    threshold=6
    if(theta>threshold): # if robot is too far to the right turn left
        print("left")
       
    if(theta<-threshold): # if robot is too far to the left turn right
        print("right")
       
    if(abs(theta)<threshold): # not sure but go straight
        print "straight"
    theta=0


    cv2.imshow("Edged", edged)
    cv2.imshow("Masked_Edges", masked_edges)
    #cv2.imshow("Lines", lines)
    #cv2.imshow("Lanes", lines_edges)
    key = cv2.waitKey(1) & 0xFF
    rawCapture.truncate(0)
   
    # terminate if 'q' key hit
    if key == ord("q"):
        break


