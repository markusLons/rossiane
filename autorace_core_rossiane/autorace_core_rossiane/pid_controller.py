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

class Controller(Node):

	def __init__(self):
		super().__init__('controller')
		self.declare_parameters(
            namespace='',
            parameters=[
            ('R', 0.0325),
            ('L', 0.1),
			('Kp', 1.0),
			('Ki', 0.01),
			('Kd', 0.01),
			('dt', 0.01),
			('desiredV', 0.01),
			('arrive_distance', 1.0),
        ])
		self.R = self.get_parameter("R").get_parameter_value().double_value					# in meter
		self.L = self.get_parameter("L").get_parameter_value().double_value					# in meter
		self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
		self.Ki = self.get_parameter("Ki").get_parameter_value().double_value
		self.Kd = self.get_parameter("Kd").get_parameter_value().double_value
		self.dt = self.get_parameter("dt").get_parameter_value().double_value				# in second
		self.desiredV = self.get_parameter("desiredV").get_parameter_value().double_value
		self.arrive_distance = self.get_parameter("arrive_distance").get_parameter_value().double_value


		self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
		self.subscription = self.create_subscription(Image, '/depth/image', self.callback, 10)
		self.br = CvBridge()
		self.subscription # prevent unused variable warn

		self.current = 0
		self.goal = 0

		self.E = 0   # Cummulative error
		self.old_e = 0  # Previous error
		   
	def callback(self, msg):
		dsensorImage = msg
		# current_frame = self.br.imgmsg_to_cv2(dsensorImage, "32FC1")
		# cv2.imshow('camera', current_frame[100:280, 200:650])
		# cv2.waitKey(1)
		# velMsg = Twist()			
		# self.publisher_.publish(velMsg)
          
	def uniToDiff(self, v, w):
		vR = (2*v + w*self.L)/(2*self.R)
		vL = (2*v - w*self.L)/(2*self.R)
		return vR, vL
		
	def diffToUni(self, vR, vL):
		v = self.R/2*(vR+vL)
		w = self.R/self.L*(vR-vL)
		return v, w

	def iteratePID(self):
		# Difference in x and y
		d_x = self.goal.x - self.current.x
		d_y = self.goal.y - self.current.y

		# Angle from robot to goal
		g_theta = np.arctan2(d_y, d_x)

		# Error between the goal angle and robot angle
		alpha = g_theta - self.current.theta
		# alpha = g_theta - math.radians(90)
		e = np.arctan2(np.sin(alpha), np.cos(alpha))

		e_P = e
		e_I = self.E + e
		e_D = e - self.old_e

		# This PID controller only calculates the angular
		# velocity with constant speed of v
		# The value of v can be specified by giving in parameter or
		# using the pre-defined value defined above.
		w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D

		w = np.arctan2(np.sin(w), np.cos(w))

		self.E = self.E + e
		self.old_e = e
		v = self.desiredV

		return v, w

	def fixAngle(self, angle):
		return np.arctan2(np.sin(angle), np.cos(angle))

	def makeAction(self, v, w):
		x_dt = v*np.cos(self.current.theta)
		y_dt = v*np.sin(self.current.theta)
		theta_dt = w

		self.current.x = self.current.x + x_dt * self.dt
		self.current.y = self.current.y + y_dt * self.dt
		self.current.theta = self.fixAngle(
			self.current.theta + self.fixAngle(theta_dt * self.dt))
		return

	def isArrived(self):
		# print("Arrive check:", str(abs(self.current.x - self.goal.x)),
		#       str(abs(self.current.y - self.goal.y)))
		current_state = np.array([self.current.x, self.current.y])
		goal_state = np.array([self.goal.x, self.goal.y])
		difference = current_state - goal_state

		distance_err = difference @ difference.T
		if distance_err < self.arrive_distance:
			return True
		else:
			return False

	def runPID(self, parkour=None):
		x = [self.current.x]
		y = [self.current.y]
		theta = [self.current.theta]
		while(not self.isArrived()):
			v, w = self.iteratePID()
			self.makeAction(v, w)
			x.append(self.current.x)
			y.append(self.current.y)
			theta.append(self.current.theta)
			if parkour:

				parkour.drawPlot(x, y, theta)
			# time.sleep(self.dt)

			# Print or plot some things in here
			# Also it can be needed to add some max iteration for
			# error situations and make the code stable.
			# print(self.current.x, self.current.y, self.current.theta)
		return x, y, theta
	
def trackRoute(start, targets):
    current = start
    x = []
    y = []
    theta = []
    for target in targets:
        controller = Controller(current, target)
        x_, y_, theta_ = controller.runPID()
        x.extend(x_)
        y.extend(y_)
        theta.extend(theta_)
        current = controller.current
    return x, y, theta

def main(args=None):
    rclpy.init(args=args)
    robot_app = Controller()
    rclpy.spin(robot_app)
	
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
