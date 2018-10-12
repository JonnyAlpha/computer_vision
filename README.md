# Computer Vision
This is a repository to store my current work on computer vision.
For PiWars2018 our team invested in a PiBorg MonsterBorg the chassis used in Forumla Pi. During the development of the code and mechanics for the varios PiWars challenges I came across a program by PiBorg to follow a line using a Raspberry Pi Camera. The program used OpenCV to enable computer vision and this insight into a different level of robotics really excited me, as it was an area that I had never looked into before.
I am currently working on the 'Maze Navigation' but this repository will contain all of my pieces of code relating to my study.

The first approach that I am taking is to detect the lines created by the walls of the maze, especially where they meet the floor, and use this as a handrail for the robot to follow.

Information on the various files in the repository:

maze_navigation.py
This is the first working program using static images to demonstrate the concept of line detection. 

maze_navigation_video.py
This program uses a recorded video as the source of the image data to work on. The video was recorded using a Raspberry Pi Camera

maze_navigation_live.py
This program uses live video from the Raspberry Pi Camera.
