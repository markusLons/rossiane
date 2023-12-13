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
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class Controller(Node):

	def __init__(self):
		super().__init__('controller')
		self.declare_parameters(
            namespace='',
            parameters=[
			('Kp', 1.0),
			('Ki', 0.01),
			('Kd', 0.01),
			('desiredV', 0.01),
        ])
		self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
		self.subscription = self.create_subscription(Float64, '/detect/lane', self.iteratePID, 10)
		self.br = CvBridge()
		self.twist = Twist()
		self.subscription # prevent unused variable warn

		self.E = 0   # Cummulative error
		self.old_e = 0  # Previous error

		
	def iteratePID(self, msg):
		self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
		self.Ki = self.get_parameter("Ki").get_parameter_value().double_value
		self.Kd = self.get_parameter("Kd").get_parameter_value().double_value
		self.desiredV = self.get_parameter("desiredV").get_parameter_value().double_value
		
		e = (390 - msg.data)/100
		# self.get_logger().info('Message data: %lf' % msg.data)
		# self.get_logger().info('Error: %lf' % e)
		e_P = e
		e_I = self.E + e
		e_D = e - self.old_e

		# This PID controller only calculates the angular
		# velocity with constant speed of v
		# The value of v can be specified by giving in parameter or
		# using the pre-defined value defined above.
		w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D
		if w > 5: w = 5
		if w < -5: w = -5
		#w = np.arctan2(np.sin(w), np.cos(w))

		self.E = self.E + e
		self.old_e = e
		v = self.desiredV
		self.twist.linear.x = v
		self.twist.angular.z = float(w)
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
