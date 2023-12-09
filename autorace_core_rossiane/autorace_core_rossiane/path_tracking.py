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

class DetectLane(Node):

	def __init__(self):
		super().__init__('autorace_core_rossiane')
		self.declare_parameters(
            namespace='',
            parameters=[
            ('white/hue_l', 0),
            ('white/hue_h', 179),
            ('white/saturation_l', 0),
            ('white/saturation_h', 70),
            ('white/lightness_l', 105),
			('white/lightness_h', 255),
			('yellow/hue_l', 10),
			('yellow/hue_h', 127),
			('yellow/saturation_l', 70),
			('yellow/saturation_h', 255),
			('yellow/lightness_l', 95),
			('yellow/lightness_h', 255),
        ])
		self.hue_white_l = self.get_parameter("white/hue_l").get_parameter_value().integer_value
		self.hue_white_h = self.get_parameter("white/hue_h").get_parameter_value().integer_value
		self.saturation_white_l = self.get_parameter("white/saturation_l").get_parameter_value().integer_value
		self.saturation_white_h = self.get_parameter("white/saturation_h").get_parameter_value().integer_value
		self.lightness_white_l = self.get_parameter("white/lightness_l").get_parameter_value().integer_value
		self.lightness_white_h = self.get_parameter("white/lightness_h").get_parameter_value().integer_value
		self.hue_yellow_l = self.get_parameter("yellow/hue_l").get_parameter_value().integer_value
		self.hue_yellow_h = self.get_parameter("yellow/hue_h").get_parameter_value().integer_value
		self.saturation_yellow_l = self.get_parameter("yellow/saturation_l").get_parameter_value().integer_value
		self.saturation_yellow_h = self.get_parameter("yellow/saturation_h").get_parameter_value().integer_value
		self.lightness_yellow_l = self.get_parameter("yellow/lightness_l").get_parameter_value().integer_value
		self.lightness_yellow_h = self.get_parameter("yellow/lightness_h").get_parameter_value().integer_value

		self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
		self.subscription = self.create_subscription(Image, '/color/image_projected_compensated', self.callback, 10)
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
    robot_app = DetectLane()
    rclpy.spin(robot_app)
	
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
