import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

class TidyScene(Node):
    def __init__(self, deltaTime):
        super().__init__('TidyScene')

        # Subscribe to the camera topic.
        self.create_subscription(Image, "/limo/depth_camera_link/image_raw", self.cameraCallback, 10)

        # Start a timer that calls the onTick function every X seconds.
        self.timer = self.create_timer(deltaTime, self.onTick)

        # Create Bridge object. Used to convert between ROS2 and OpenCV Images.
        self.cvBridge = CvBridge()

    def onTick(self):
        bamting = True # Haven't implemented Yet.

    def cameraCallback(self, data):
        # Convert ROS Image message to OpenCV image
        frame = self.cvBridge.imgmsg_to_cv2(data, desired_encoding = "bgr8")

        # Convert image to HSV
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for range of colours (HSV low values, HSV high values)
        frameMask = cv2.inRange(frameHSV,(0, 150, 50), (255, 255, 255)) # orange

        contours, hierarchy = cv2.findContours(frameMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Sort by area (keep only the biggest one)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        frameContours = cv2.drawContours(frame, contours, 0, (255, 255, 0), 20)

        # show the cv images
        frameContoursSmall = cv2.resize(frameContours, (0,0), fx=0.4, fy=0.4) # reduce image size

        # Show the image.
        cv2.imshow("Image Window", frameContoursSmall)
        cv2.waitKey(1)


def main(args = None):

    # Initialize ROS2. Must be done before nodes are created.
    rclpy.init(args = args)

    # Define the tickrate of the tidyScene Program.
    deltaTime = 1 / 2

    # Create Instance of our TidyScene Program.
    tidyScene = TidyScene(deltaTime)

    # Executes work on the node until its shut down.
    rclpy.spin(tidyScene)

    # Destroy the node once it has finished and Shutdown ROS2.
    tidyScene.destroy_node()
    rclpy.shutdown()
 

if __name__ == '__main__':
    main()

# ROBOT PLAN
# 
# Need robot to find cubes, and push them to any wall.
# Use LIDAR & Camera Feed to determine if a cube is at the wall or not.
#
#
#
#
#
#
#
#