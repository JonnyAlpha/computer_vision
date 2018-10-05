# https://towardsdatascience.com/finding-driving-lane-line-live-with-opencv-f17c266f15db
# Maze navigation using canny edge detection and houghlinesP transform
import cv2
import numpy as np

image = cv2.imread("/home/pi/opencv/maze_test_images/maze1.png")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
kernel_size = 5
blur_gray = cv2.GaussianBlur(gray,(kernel_size,kernel_size),0)
low_threshold = 50
high_threshold = 150

edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

# create a mask of the edges image using cv2.filpoly()
mask = np.zeros_like(edges)
ignore_mask_color = 255

# define the Region of Interest (ROI) - source code sets as a trapezoid for roads
imshape = image.shape

vertices = np.array([[(0,imshape[0]),(100, 420), (1590, 420),(imshape[1],imshape[0])]], dtype=np.int32)

cv2.fillPoly(mask, vertices, ignore_mask_color)
masked_edges = cv2.bitwise_and(edges, mask)

# mybasic ROI bounded by a blue rectangle

#ROI = cv2.rectangle(image,(0,420),(1689,839),(0,255,0),3)

# define the Hough Transform parameters
rho = 2 # distance resolution in pixels of the Hough grid
theta = np.pi/180 # angular resolution in radians of the Hough grid
threshold = 15     # minimum number of votes (intersections in Hough grid cell)
min_line_length = 40 #minimum number of pixels making up a line
max_line_gap = 30    # maximum gap in pixels between connectable line segments

# make a blank the same size as the original image to draw on
line_image = np.copy(image)*0 

# run Hough on edge detected image
lines = cv2.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]),min_line_length, max_line_gap)

for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)

# draw the line on the original image 
lines_edges = cv2.addWeighted(image, 0.8, line_image, 1, 0)
#return lines_edges

#coord = np.where(np.all(lines_edges == (255,0,0), axis=-1))
#print zip(coord[0], coord[1])


cv2.imshow("original", image)
cv2.waitKey(0)

cv2.imshow("edges", edges)
cv2.waitKey(0)

cv2.imshow("detected", lines_edges)
cv2.waitKey(0)

cv2.imwrite("lanes_detected.jpg", lines_edges)
cv2.destroyAllWindows()


            


