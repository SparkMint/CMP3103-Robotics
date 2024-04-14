import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

class TidyScene(Node):
    def __init__(self, deltaTime):
        super().__init__('TidyScene')

        self.targetObject = None
        self.dt = deltaTime
        self.cubeSize = 0.2
        self.distance = 0

        # Subscribe to the camera and laser topic.
        self.create_subscription(Image, "/limo/depth_camera_link/image_raw", self.cameraCallback, 1 / deltaTime)
        self.create_subscription(LaserScan, "/scan", self.laserScanCallback, 1 / deltaTime)

        # Create a timer
        self.timer = self.create_timer(deltaTime, self.onTick)

        # Create Bridge object. Used to convert between ROS2 and OpenCV Images.
        self.cvBridge = CvBridge()

    def cameraCallback(self, data):
        # Convert ROS Image message to OpenCV image
        frame = self.cvBridge.imgmsg_to_cv2(data, desired_encoding = "bgr8")

        # Convert image to HSV
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for range of colours (HSV low values, HSV high values)
        frameMask = cv2.inRange(frameHSV,(0, 150, 50), (255, 255, 255)) # orange

        # Find all Contours
        contours, hierarchy = cv2.findContours(frameMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        frameContours = cv2.drawContours(frame, contours, -1, (255, 255, 0), 20)

        # show the cv images
        frameContoursSmall = cv2.resize(frameContours, (0,0), fx=0.4, fy=0.4) # reduce image size

        # Show the image.
        cv2.imshow("Image Window", frameContoursSmall)
        cv2.waitKey(1)

        # Reset our target for this frame.
        self.targetObject = None

        # Find our target object
        self.findTargetObject(contours, data)

        if not self.targetObject:
            print("No target found")
            # TODO: Turn around until we find a valid one.
            return
            
        # Get the X position of the contour
        cx = int(self.targetObject['m10']/self.targetObject['m00'])

        # We use the camera's data to find the width of it.

        # If the object's center is to the left, turn left.
        if cx < data.width / 3:
            #print("I wanna turn left")
            bamting = False
        # If the object's center is to the right, turn right.
        elif cx >= 2 * data.width / 3:
            #print("I wanna turn right")
            bamting = False
        # The object is within 100px of the camera's center.
        else: 
            #print("Im looking at my target")
            bamting = True

    def laserScanCallback(self, data):
        bamting = True

    def onTick(self):
        bamting = True

    def findTargetObject(self, contours, data):
        currentContour = 0
        # If we can see a contour, rotate towards it
        for contour in contours:
            print("Checking validity of contour ", currentContour)
            currentContour += 1
            M = cv2.moments(contour)
            if M['m00'] <= 0:
                continue
            
            # Get the Height of the contour center
            cy = int(M['m01']/M['m00'])

            # If the object is higher than our robot. Its probably out of reach. Ignore it.
            if cy <= data.height / 3:
                print("Object centre too high!")
                continue

            # TODO: Check if this box is against a wall or not. LIDAR is your best bet.
            self.calculateDistance(contour, data)
            print("Object distance = ", self.distance)

            # If we got here, We want to head to this cube.
            self.targetObject = M
            #print("Found target Object")
            break

    def calculateDistance(self, contour, data):
        # Get the bounding box of this contour.
        # Then get the width of it. We can use this to calculate the distance.
        # https://www.geeksforgeeks.org/realtime-distance-estimation-using-opencv-python/
        boundingBox = cv2.minAreaRect(contour)
        apparentWidth = boundingBox[1][0]
        self.distance = (self.cubeSize * data.width / apparentWidth)

def main(args = None):

    # Initialize ROS2. Must be done before nodes are created.
    rclpy.init(args = args)

    deltaTime = 1 / 10

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