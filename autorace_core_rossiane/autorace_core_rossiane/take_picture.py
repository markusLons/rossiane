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
import matplotlib.pyplot as plt
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class PublisherSubscriber(Node):

	def __init__(self):
		super().__init__('robot_app')
		self.subscription = self.create_subscription(Image, '/color/image', self.callback, 10)
		self.br = CvBridge()
		self.count = 1
		self.i = 1
		self.subscription # prevent unused variable warn
		   
	def callback(self, msg):
		if self.i % 10 != 0:
			self.i += 1
			return
		else:
			self.i = 1
		dsensorImage = msg
		current_frame = self.br.imgmsg_to_cv2(dsensorImage, "bgr8")
		filename = f"dataset/empty/empty{self.count}.png"
		cv2.imwrite(filename, current_frame)
		self.get_logger().info('Picture: %s' % filename)
		cv2.waitKey(1)
		self.count += 1

def main(args=None):
    rclpy.init(args=args)
    robot_app = PublisherSubscriber()
    rclpy.spin(robot_app)
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()