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
import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

class PublisherSubscriber(Node):

	def __init__(self):
		super().__init__('robot_app')
		self.publisher_ = self.create_publisher(Bool, '/start', 1)
		self.subscription = self.create_subscription(Image, '/color/image', self.callback, 1)
		self.br = CvBridge()
		self.msg = Bool()
		self.start = False
		self.subscription # prevent unused variable warn
		path = os.path.join(
			get_package_share_directory('autorace_core_rossiane'),
			'calibration',
			'best.pt'
			)
		self.model = YOLO(path)
		self.names = self.model.names

	def callback(self, msg):
		dsensorImage = msg
		current_frame = self.br.imgmsg_to_cv2(dsensorImage, "bgr8")
		#YOLO
		results = self.model.predict(current_frame, show = True)
		for c in results[0]:
			value = c.boxes.cls.item()  
			self.get_logger().info(self.names[int(value)])
			if int(value) == 1 and self.start == False:
				self.msg.data = True
				self.start = True
				self.publisher_.publish(self.msg)
			elif int(value) == 0:
				pass
			
			box_coordinates = c.boxes.xyxy[0].cpu().numpy()  
			self.get_logger().info(f"Upper-left coordinates: ({box_coordinates[0]}, {box_coordinates[1]})")
			self.get_logger().info(f"Lower-right coordinates: ({box_coordinates[2]}, {box_coordinates[3]})")

def main(args=None):
    rclpy.init(args=args)
    robot_app = PublisherSubscriber()
    rclpy.spin(robot_app)
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()