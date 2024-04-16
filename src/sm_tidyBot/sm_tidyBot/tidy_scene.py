import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from cv_bridge import CvBridge
import cv2
from enum import Enum
import time

# The robot has three states that it uses to push the cubes
class RobotState(Enum):
    LookForTarget = 1 # Turn until we find a target, if we find a valid one. Point towards it.
    PushTarget = 2 # Push the target towards the wall.
    BackUpFromTarget = 3 # Back away from the target as we have pushed it to the wall Successfully.

class TidyScene(Node):

    def __init__(self, sampleRate):
        super().__init__('TidyScene')

        self.sampleRate = sampleRate

        # Extra Variables
        self.robotState = RobotState.LookForTarget

        self.targetObjectIndex = 0
        self.distanceFromTarget = 0
        self.pushForwardTime = 0
        self.backUpTime = 0

        # Twist Variables
        self.angularVelocity = 0.0
        self.linearVelocity = 0.0
        self.twist = None

        # Camera Variables
        self.contourFrame = None
        self.depthFrame = None
        self.cameraData = None

        self.contours = None
        self.targetContour = None
        self.targetContourX = 0
        self.targetContourY = 0

        # Laser Variables
        self.laserData = None

        # Subscribe to the camera and laser topic.
        self.create_subscription(Image, "/limo/depth_camera_link/image_raw", self.cameraCallback, sampleRate)
        self.create_subscription(Image, "/limo/depth_camera_link/depth/image_raw", self.depthCallback, sampleRate)
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

        # Convert ROS Image message to an OpenCV RGB image
        self.contourFrame = self.cvBridge.imgmsg_to_cv2(cameraData, "bgr8")

        # Convert image to HSV
        # Create mask for range of colours (HSV low values, HSV high values)
        # Find all Contours
        frameHSV = cv2.cvtColor(self.contourFrame, cv2.COLOR_BGR2HSV)
        frameMask = cv2.inRange(frameHSV,(0, 150, 50), (255, 255, 255))
        self.contours, hierarchy = cv2.findContours(frameMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw our contours onto the frame and reduce the size
        if self.targetObjectIndex <= len(self.contours) - 1:      
            frameContours = cv2.drawContours(self.contourFrame, self.contours, self.targetObjectIndex, (255, 255, 0), 20)
            frameContoursSmall = cv2.resize(frameContours, (0,0), fx=0.4, fy=0.4)
            cv2.imshow("Image Window", frameContoursSmall)
        else:
            noContourFrameSmall = cv2.resize(self.contourFrame, (0,0), fx=0.4, fy=0.4)
            cv2.imshow("Image Window", noContourFrameSmall)
            
        cv2.waitKey(1)

    def depthCallback(self, depthData):
        # Convert ROS Image message to an OpenCV Depth frame.
        self.depthFrame = self.cvBridge.imgmsg_to_cv2(depthData, "32FC1")

    def laserScanCallback(self, laserData):
        self.laserData = laserData

    def onTick(self):
        # Both data types need to be present to decide what to do.
        if self.cameraData is None or self.laserData is None or self.depthFrame is None:
            return
        
        # Look for suitable target objects.
        self.selectTargetContour()

        # Look For Target State
        #
        # In this state, we react to the data provided by the target contour selection function found above.
        # If there is a target, we want to head to it and line ourselves up to it. If not, we want to look around
        # so the target contour selection function can run each perceived object through it.
        if self.robotState is RobotState.LookForTarget:
            self.lookForTargetBehavior()
  
        # Push Target State
        #        
        # Since the robot loses vision of the cube when pushing it, we want to check every now and again that
        # we haven't lost the cube. So we apply a "Two Steps forward, one step back" approach. Where the robot will
        # Push the cube forward, then move back a bit and check that they haven't lost the cube.
        # We keep performing this until we get close enough to the wall.
        if self.robotState is RobotState.PushTarget:
            self.pushBehavior()

        # Back up State
        #
        # This is our "One step back" state. After doing this, we want to return to looking for a valid target.
        # If the cube isn't against the wall, the robot will see it and return to pushing.
        if self.robotState is RobotState.BackUpFromTarget:
            self.backUpBehavior()

        # Drive the robot
        self.driveRobot()


    # BEHAVIOR FUNCTIONS
    def selectTargetContour(self):
        # Reset our target for this tick.
        self.targetContour = None
        self.targetContourX = 0
        self.targetContourY = 0

        # Sort by area. The biggest (Closest ones) should take priority
        self.contours = sorted(self.contours, key=cv2.contourArea, reverse=True)

        # If we can see a contour, rotate towards it
        for cIndex, contour in enumerate(self.contours):
            M = cv2.moments(contour)
            if M['m00'] <= 0:
                continue
            
            # Get the Width and Height of the contour
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # If the object is higher than our robot. Its probably out of reach. Ignore it.
            if cy <= self.cameraData.height / 3:
                continue

            # Get the distance to our object as well as the angle required to get to it.
            distanceFromContour = self.depthFrame[cy][cx]

            # Find the angle in degrees using the horizontal center of the
            # chosen contour, the centre of the camera and the FOV (Seems to be 90 degrees)
            # https://github.com/IntelRealSense/librealsense/issues/5553
            laserAngleDeg = int(((cx - self.cameraData.width / 2) / self.cameraData.width / 2) * (90 / 2))

            # Determine the laser segment we want to get our wall distance reading from.
            # It should be in the direction of the cube we are targeting.
            # We can then get a distance between the object and the wall.
            # TODO: Update this to use a small range
            wallDistance = self.laserData.ranges[int(len(self.laserData.ranges) / 2) - laserAngleDeg]
            objectToWallDistance = wallDistance - distanceFromContour

            # Check if the distance between the object and the wall is less than the specified
            # amount. If it is. We have already pushed that to the wall. Ignore it!
            if objectToWallDistance < 0.:
                continue

            # Changes the colour of the target laser in RVIZ2. Used for debug purposes.
            self.laserData.intensities[int(len(self.laserData.ranges) / 2) - laserAngleDeg] = 100
            self.publishLaserScan.publish(self.laserData)

            # If we got here. We have found a valid target. Select this one and return.
            self.targetObjectIndex = cIndex
            self.targetContour = contour
            self.distanceFromTarget = distanceFromContour
            self.targetContourX = cx
            self.targetContourY = cy

            print("INFO")
            print("Object Distance = ", self.distanceFromTarget)
            print("Wall Distance = ", wallDistance)
            print("Object to Wall Distance = ", objectToWallDistance)

            return
        
        self.targetObjectIndex = 999

    def lookForTargetBehavior(self):
        print("Target Dist:", self.distanceFromTarget)
        # Check if we have a target to turn towards
        if self.targetContour is None:     
            self.angularVelocity = 0.75
        else:
            if self.distanceFromTarget > 0.5: # Move Towards our target
                self.linearVelocity = 0.2

            # If the object's center is to the left, turn left.
            if self.targetContourX < self.cameraData.width / 2.25:
                self.angularVelocity = 0.2

            # If the object's center is to the right, turn right.
            elif self.targetContourX > 2 * self.cameraData.width / 2.25:
                self.angularVelocity = -0.2

            # The object is in the center of our camera's view. Switch to the push state.
            elif self.distanceFromTarget <= 0.5: 
                self.pushForwardTime = 3
                self.backUpTime = 2
                self.robotState = RobotState.PushTarget

    def pushBehavior(self):
        self.linearVelocity = 0.4

        self.pushForwardTime -= 1 / self.sampleRate

        if self.pushForwardTime <= 0:   
            self.robotState = RobotState.BackUpFromTarget

    def backUpBehavior(self):
        self.linearVelocity = -0.4

        self.backUpTime -= 1 / self.sampleRate

        if self.backUpTime <= 0: 
           self.robotState = RobotState.LookForTarget

    def driveRobot(self):
        # Publish any inputs provided by the bot.
        self.twist = Twist()
        self.twist.angular.z = self.angularVelocity
        self.twist.linear.x = self.linearVelocity
        self.publishTwist.publish(self.twist)

        # Refresh the robots velocity after since it will receive instructions every tick.
        self.angularVelocity = 0.0
        self.linearVelocity = 0.0

def main(args = None):

    # Initialize ROS2. Must be done before nodes are created.
    rclpy.init(args = args)

    # Update 10 times per second.
    sampleRate = 30

    # Create Instance of our TidyScene Program.
    tidyScene = TidyScene(sampleRate)

    # Executes work on the node until its shut down.
    rclpy.spin(tidyScene)

    # Destroy the node once it has finished and Shutdown ROS2.
    tidyScene.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()