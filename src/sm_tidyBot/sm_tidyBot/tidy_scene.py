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
        print("")

    def cameraCallback(self, data):
        # Convert ROS Image message to OpenCV image
        currentFrame = self.cvBridge.imgmsg_to_cv2(data, desired_encoding = "bgr8")

        # Show the image.
        cv2.imshow("Image Window",currentFrame)
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