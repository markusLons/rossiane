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
import time
import matplotlib.pyplot as plt
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Empty, Bool, UInt16, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan

class Controller(Node):

	def __init__(self):
		super().__init__('controller')
		self.declare_parameters(
            namespace='',
            parameters=[
			('Kp', 1.0),
			('Kv', 0.01),
			('desiredV', 0.01),
        ])
		self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
		self.subscription = self.create_subscription(Float64, '/detect/lane', self.iteratePID, 5)
		self.subscription2  = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
		self.subscription6  = self.create_subscription(UInt16, '/parking', self.callback_parking, 1)
		self.commands_subscription  = self.create_subscription(String, '/pid', self.callback, 1)
		self.br = CvBridge()
		self.twist = Twist()
		self.commands = ""
		self.obstacles = False
		self.pedestrian = False
		self.parking = False
		self.left_or_right = 0
		self.startTime = 0.0
		self.left = 0
		self.subscription # prevent unused variable warn
		self.commands_subscription

		self.E = 0   # Cummulative error
		self.old_e = 0  # Previous error

	def callback(self, msg):
		self.commands = msg.data

	def callback_parking(self, msg):
		if msg.data == 1:
			self.left_or_right = -1
		else:
			self.left_or_right = 1
		self.parking = True

	def scan_callback(self, msg):
		self.ranges = msg.ranges
		
	def iteratePID(self, msg):
		self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
		self.Kv = self.get_parameter("Kv").get_parameter_value().double_value
		self.desiredV = self.get_parameter("desiredV").get_parameter_value().double_value

		if self.commands == 'crossroad':
			self.desiredV = 0.18
			if self.startTime == 0.0:
				self.startTime = time.time()
			if time.time() - self.startTime > 26.0:
				self.startTime = 0.0
				self.commands = ''

		if self.commands == 'parking':
			self.desiredV = 0.18
			self.Kv = 0.0
			if self.startTime == 0.0:
				self.startTime = time.time()
			if time.time() - self.startTime > 30.0:
				self.startTime = 0.0
				self.commands = ""

		e = (394 - msg.data)/100
		# self.get_logger().info('Error: %lf' % e)
		e_P = e
		e_I = self.E + e
		e_D = e - self.old_e

		w = self.Kp*e_P # + self.Kl*e_I + self.Kl*e_D
		if w > 5: w = 5
		if w < -5: w = -5
		#w = np.arctan2(np.sin(w), np.cos(w))
		
		self.E = self.E + e
		self.old_e = e
		v = self.desiredV - self.Kv*abs(e_P)
		self.twist.linear.x = v
		self.twist.angular.z = float(w)

		if self.commands == "obstacles":
			if any(v < 0.26 for v in self.ranges[95:105]) and self.left == 0:
				self.left = 1
			if self.left == 1:
				self.twist.angular.z = 1.0
				if any(v < 0.4 for v in self.ranges[335:345]):
						self.left = 2
			if self.left == 2:
				self.twist.angular.z = -1.0
				if self.startTime == 0.0:
					self.startTime = time.time()
				if time.time() - self.startTime > 3.0:
					if any(v < 0.4 for v in self.ranges[8:20]):
						self.left = 3
			if self.left == 3:
				self.twist.angular.z = 0.85
				self.startTime = 0.0

			self.twist.linear.x = 0.21

		if self.parking == True:
			if time.time() - self.startTime > 22.0 + self.left_or_right/5:
				pass
			elif time.time() - self.startTime > 16.9 + self.left_or_right/5:
				self.twist.linear.x = 0.18
				self.twist.angular.z = 0.0
			elif time.time() - self.startTime > 13.5 + self.left_or_right/5:
				self.twist.linear.x = -0.2
				self.twist.angular.z = self.left_or_right * 0.80 + 0.04
			elif time.time() - self.startTime > 11.5:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
			elif time.time() - self.startTime > 7.5 + self.left_or_right/4:
				self.twist.linear.x = 0.2
				self.twist.angular.z = self.left_or_right * 0.7

		if self.commands == 'pedestrian':
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
			self.publisher_.publish(self.twist)
		
		if self.commands == "tunnel":
			if any(v < 0.5 for v in self.ranges[4:6]):
				self.twist.angular.z = 1.0
				self.twist.linear.x = 0.15


		self.publisher_.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    robot_app = Controller()
    rclpy.spin(robot_app)
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
