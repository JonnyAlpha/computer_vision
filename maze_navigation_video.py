# Original ideas from https://towardsdatascience.com/finding-driving-lane-line-live-with-opencv-f17c266f15db
# Maze navigation using canny edge detection and houghlinesP transform
# v1.0
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import math

# setup variables
theta = 0


# setup camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))
# warmup camera
time.sleep(0.1)

# capture and work with the video

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        
        image = frame.array
        
        # rotate the original image 180 degrees
        (h, w) = image.shape[:2]
        center = (w/2, h/2)
        M = cv2.getRotationMatrix2D(center, 180, 1.0)
        rotated =cv2.warpAffine(image,M,(w,h))
         
        # apply gray scaling
        gray = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)
        # apply blurring
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(gray,(kernel_size,kernel_size),0)

        # edges  
        low_threshold = 50
        high_threshold = 150
        edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

        # create a mask of the edges image using cv2.filpoly()
        mask = np.zeros_like(edges)
        ignore_mask_color = 255

        # define the Region of Interest (ROI) - source code sets as a trapezoid for roads
        imshape = image.shape

        vertices = np.array([[(0,imshape[0]),(450, 320), (500, 320),(imshape[1],imshape[0])]], dtype=np.int32)

        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_edges = cv2.bitwise_and(edges, mask)


        # define the Hough Transform parameters
        rho = 2 # distance resolution in pixels of the Hough grid
        theta2 = np.pi/180 # angular resolution in radians of the Hough grid
        threshold = 15     # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 40 #minimum number of pixels making up a line
        max_line_gap = 30    # maximum gap in pixels between connectable line segments

        # make a blank the same size as the original image to draw on
        line_image = np.copy(rotated)*0 

        # run Hough on masked version of edge detected image
        lines = cv2.HoughLinesP(masked_edges, rho, theta2, threshold, np.array([]),min_line_length, max_line_gap)

        
        for line in lines:
                for x1,y1,x2,y2 in line:
                    cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)
                    theta=theta+math.atan2((y2-y1),(x2-x1))    
                    #print(lines)
            
        # draw the line on the original image 
        lines_edges = cv2.addWeighted(rotated, 0.8, line_image, 1, 0)
        #return lines_edges

        threshold=6
        if(theta>threshold): # if robot is too far to the right turn left
               print("left")
       
        if(theta<-threshold): # if robot is too far to the left turn right
               print("right")
       
        if(abs(theta)<threshold): # not sure but go straight
               print "straight"
        theta=0


        cv2.imshow("Lanes", lines_edges)
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
   
        # terminate if 'q' key hit
        if key == ord("q"):
            break
