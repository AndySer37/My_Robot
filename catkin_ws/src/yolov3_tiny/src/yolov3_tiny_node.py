#!/usr/bin/env python

import numpy as np
import cv2
import roslib
import rospy
import time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import rospkg
from nav_msgs.msg import Path
from cv_bridge import CvBridge, CvBridgeError
import torch
from torch.utils.data import DataLoader
from torch.autograd import Variable
import torch.optim as optim
import os 
import message_filters
import tqdm

from models import *
from utils.utils import *
from utils.parse_config import *

class yolov3_node(object):
	def __init__(self):
		model_type = "yolov3-tiny"
		# config_path = "/home/andyser/code/David_dream/My_Robot/catkin_ws/src/yolov3_tiny/src"
		config_path = "/home/nvidia/code/My_Robot/catkin_ws/src/yolov3_tiny/src"
		self.prob_threshold = 0.85
		self.img_size = 416
		self.cv_bridge = CvBridge() 
		
		self.objects = []
		data_config_path = "config/subt.data"
		model_config_path = "config/yolov3-tiny.cfg"
		cls_path = "data/subt.names"

		data_config = parse_data_config(os.path.join(config_path, data_config_path))

		self.num_classes = int(data_config["classes"])
		self.model = Darknet(os.path.join(config_path, model_config_path))
		self.cuda = torch.cuda.is_available()

		if model_type == "yolov3-tiny":
			self.model.load_state_dict(torch.load("/home/nvidia/Desktop/model_yolo.pth"))
		if self.cuda:
			self.model = self.model.cuda()

		self.model.eval()
		self.Tensor = torch.cuda.FloatTensor if self.cuda else torch.FloatTensor
		self.class_names = load_classes(os.path.join(config_path, cls_path))


		#### Publisher
		self.image_pub = rospy.Publisher("/predict_img", Image, queue_size = 1)
		self.mask_pub = rospy.Publisher("/predict_mask", Image, queue_size = 1)

		### msg filter 
		self.is_compressed = False

		image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)

	def callback(self, img_msg):
		try:
			if self.is_compressed:
				np_arr = np.fromstring(img_msg.data, np.uint8)
				cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			else:
				cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		predict_img = self.predict(cv_image)
		try:
			self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(predict_img, "bgr8"))
		except CvBridgeError as e:
			print(e)


	def predict(self, img):
		# Preprocessing

		# image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
		image, _ = self.pad_to_square(img, 127)
		image = cv2.resize(image, (self.img_size, self.img_size), interpolation=cv2.INTER_NEAREST)
		image = np.transpose(image, (2, 0, 1))
		image = torch.from_numpy(image).float() / 255.0
		image = Variable(image.unsqueeze(0))
		image = Variable(image.type(self.Tensor), requires_grad=False)
		time = rospy.get_time()
		with torch.no_grad():
			detections = self.model(image)
			detections = non_max_suppression(detections, conf_thres=self.prob_threshold, nms_thres=0.1)
		print(1./(rospy.get_time() - time))
		print detections
		for detection in detections:
			if detection is not None:
				# Rescale boxes to original image
				detection = rescale_boxes(detection, self.img_size, img.shape[:2])

				for x1, y1, x2, y2, conf, cls_conf, cls_pred in detection:

					print("\t+ Label: %s, Conf: %.5f" % (self.class_names[int(cls_pred)], cls_conf.item()))
					cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 0), 4)

					label = "{} {:.2f}".format(self.class_names[int(cls_pred)][3:], cls_conf.numpy())
					cv2.putText(img, label,(x1, y1 - 20),cv2.FONT_HERSHEY_SIMPLEX,1,(255, 0, 255),2)

		return img# , objs


	def onShutdown(self):
		rospy.loginfo("Shutdown.")

	def pad_to_square(self, img, pad_value):
		h, w, _ = img.shape
		dim_diff = np.abs(h - w)
		# (upper / left) padding and (lower / right) padding
		pad1 = dim_diff // 2
		pad2 = dim_diff - pad1
		# Determine padding
		pad = ((pad1, pad2), (0, 0), (0, 0)) if h <= w else ((0, 0), (pad1, pad2), (0, 0))
		# Add padding
		img = np.pad(img, pad, "constant", constant_values=pad_value)

		return img, pad

if __name__ == '__main__': 
	rospy.init_node('yolov3_node',anonymous=False)
	yolov3_node = yolov3_node()
	rospy.on_shutdown(yolov3_node.onShutdown)
	rospy.spin()
