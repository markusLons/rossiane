# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class PublisherSubscriber(Node):

	def __init__(self):
		super().__init__('robot_app')
		self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
		self.subscription = self.create_subscription(Image, '/color/image_projected_compensated', self.callback, 10)
		self.previousData = []
		self.br = CvBridge()
		self.subscription # prevent unused variable warn
		   
	def callback(self, msg):
		dsensorImage = msg
		current_frame = self.br.imgmsg_to_cv2(dsensorImage, "bgr8")
		cv2.imshow('camera', current_frame)
		cv2.waitKey(1)
		velMsg = Twist()
		self.publisher_.publish(velMsg)

def main(args=None):
    rclpy.init(args=args)
    robot_app = PublisherSubscriber()
    rclpy.spin(robot_app)
	
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()