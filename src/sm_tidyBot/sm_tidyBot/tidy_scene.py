import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np

from cv_bridge import CvBridge
import cv2
from enum import Enum

class RobotState(Enum):
    LookForTarget = 1
    RotateTowardsTarget = 2
    PushTarget = 3
    MoveAwayFromWall = 4

class TidyScene(Node):

    def __init__(self, sampleRate):
        super().__init__('TidyScene')

        self.targetObject = None
        self.robotState = RobotState.LookForTarget

        # Twist Variables
        self.angularVelocity = 0
        self.linearVelocity = 0
        self.twist = None

        self.sampleRate = sampleRate
        self.cubeSize = .05
        self.distance = 0

        # Camera Variables
        self.contours = None
        self.cameraData = None

        # Laser Variables
        self.laserData = None
        self.laserAngleDeg = 0

        # Timer Variables
        # TODO: Implement Me!

        # Subscribe to the camera and laser topic.
        self.create_subscription(Image, "/limo/depth_camera_link/image_raw", self.cameraCallback, sampleRate)
        self.create_subscription(LaserScan, "/scan", self.laserScanCallback, sampleRate)

        # Create Publishers.
        self.publishLaserScan = self.create_publisher(LaserScan, "/scan", sampleRate)
        self.publishTwist = self.create_publisher(Twist, "/cmd_vel", sampleRate)

        # Create a timer
        self.timer = self.create_timer(1 / sampleRate, self.onTick)

        # Create Bridge object. Used to convert between ROS2 and OpenCV Images.
        self.cvBridge = CvBridge()

    def cameraCallback(self, cameraData):
        # Store our camera's data    
        self.cameraData = cameraData

        # Convert ROS Image message to OpenCV image
        frame = self.cvBridge.imgmsg_to_cv2(cameraData, desired_encoding = "bgr8")

        # Convert image to HSV
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for range of colours (HSV low values, HSV high values)
        frameMask = cv2.inRange(frameHSV,(0, 150, 50), (255, 255, 255)) # orange

        # Find all Contours
        self.contours, hierarchy = cv2.findContours(frameMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        frameContours = cv2.drawContours(frame, self.contours, -1, (255, 255, 0), 20)

        # show the cv images
        frameContoursSmall = cv2.resize(frameContours, (0,0), fx=0.4, fy=0.4) # reduce image size

        # Show the image.
        cv2.imshow("Image Window", frameContoursSmall)
        cv2.waitKey(1)

    def laserScanCallback(self, laserData):
        self.laserData = laserData

    def onTick(self):
        # Both data types need to be present to decide what to do.
        if self.cameraData is None or self.laserData is None:
            return
        
        # Refresh Velocities each tick.
        self.angularVelocity = float(0)
        self.linearVelocity = float(0)

        print("Robot State: ", self.robotState.name)

        # Look For Target State
        if self.robotState is RobotState.LookForTarget:
            # Reset our target for this tick.
            self.targetObject = None

            # Find our target object
            self.findTargetObject()

            if self.targetObject is None:
                self.angularVelocity = 0.3
            else:
                self.robotState = RobotState.RotateTowardsTarget

        # Rotate Towards Target State
        if self.robotState is RobotState.RotateTowardsTarget:
            self.rotateTowardsTarget()
  
        # Push Target State
        if self.robotState is RobotState.PushTarget:
            bamting = True

        # Move Away State
        if self.robotState is RobotState.MoveAwayFromWall:
            bamting = False

        # Publish any inputs provided by the bot.
        self.twist = Twist()
        self.twist.angular.z = self.angularVelocity
        self.twist.linear.x = self.linearVelocity
        self.publishTwist.publish(self.twist)


    def findTargetObject(self):
        currentContour = 0
        # If we can see a contour, rotate towards it
        for contour in self.contours:
            print("Checking validity of contour ", currentContour)
            currentContour += 1
            M = cv2.moments(contour)
            if M['m00'] <= 0:
                continue
            
            # Get the Height of the contour center
            cy = int(M['m01']/M['m00'])

            # If the object is higher than our robot. Its probably out of reach. Ignore it.
            if cy <= self.cameraData.height / 2:
                print("Object centre too high!")
                continue

            self.calculateDistance(contour)
            self.calculateAngleToTarget(M)

            laserToCheck = self.laserData.ranges[int(len(self.laserData.ranges) / 2) - self.laserAngleDeg]
            objectToWallDistance = laserToCheck - self.distance
            print("Object to Wall Distance: ", objectToWallDistance)

            # Changes the colour of the target laser in Rviz2. Used for debug purposes.
            self.laserData.intensities[int(len(self.laserData.ranges) / 2) - self.laserAngleDeg] = 100
            self.publishLaserScan.publish(self.laserData)

            # If we got here, We want to head to this cube.
            self.targetObject = contour
            #print("Found target Object")
            break

    def rotateTowardsTarget(self):
        # Get the X position of the contour
        M = cv2.moments(self.targetObject)
        cx = int(M['m10']/M['m00'])
        print("TargetXPos: ", cx)

        # If the object's center is to the left, turn left.
        if cx < self.cameraData.width / 3:
            print("Going left")
            self.angularVelocity = 0.3
        # If the object's center is to the right, turn right.
        elif cx >= 2 * self.cameraData.width / 3:
            print("Going Right")
            self.angularVelocity = -0.3
        # The object is within 100px of the camera's center.
        else: 
            self.robotState = RobotState.PushTarget

    def calculateAngleToTarget(self, M):
        # Get the X position of the contour
        cx = int(M['m10']/M['m00'])

        deltaX = cx - self.cameraData.width / 2
        laserAngleRad = np.arctan2(deltaX, self.cameraData.width / 2)
        self.laserAngleDeg = int(np.degrees(laserAngleRad))

    def calculateDistance(self, contour):
        # Get the bounding box of this contour.
        # Then get the width of it. We can use this to calculate the distance.
        # https://www.geeksforgeeks.org/realtime-distance-estimation-using-opencv-python/
        boundingBox = cv2.minAreaRect(contour)
        apparentWidth = boundingBox[1][0]
        self.distance = (self.cubeSize * self.cameraData.width / apparentWidth)

def main(args = None):

    # Initialize ROS2. Must be done before nodes are created.
    rclpy.init(args = args)

    # Update 10 times per second.
    sampleRate = 10

    # Create Instance of our TidyScene Program.
    tidyScene = TidyScene(sampleRate)

    # Executes work on the node until its shut down.
    rclpy.spin(tidyScene)

    # Destroy the node once it has finished and Shutdown ROS2.
    tidyScene.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()