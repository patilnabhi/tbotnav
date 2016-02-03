#!/usr/bin/env python

import cv2
import cv2.cv as cv
import numpy as np 

class RecognizeNumFingers:
	def __init__(self):
		self.abs_depth_dev = 14

		self.thresh_deg = 80.0

	def find(self, img):
		self.height, self.width = img.shape[:2]

		segment = self._segment_arm(img)

		(contours, defects) = self._find_hull_defects(segment)

		outimg = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

		(outimg, num_fingers) = self._detect_num_fingers(contours, defects, outimg)

		return (outimg, num_fingers)

	def _segment_arm(self, img):
		center_half = 10
		center = img[self.height/2 - center_half : self.height/2 + center_half, self.width/2 - center_half : self.width/2 + center_half]

		median_val = np.median(center)

		img = np.where(abs(img-median_val) <= self.abs_depth_dev, 128, 0).astype(np.uint8)

		kernel = np.ones((3,3), np.uint8)
		img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

		small_kernel = 3
		img[self.height/2 - small_kernel : self.height/2 + small_kernel, self.width/2 - small_kernel : self.width/2 + small_kernel] = 128

		mask = np.zeros((self.height + 2, self.width + 2), np.uint8)
		flood = img.copy()
		cv2.floodFill(flood, mask, (self.width/2, self.height/2), 255, flags=4 | (255 << 8))
		
		ret, flooded = cv2.threshold(flood, 129, 255, cv2.THRESH_BINARY)

		return flooded

	def _find_hull_defects(self, segment):
		contours, hierarchy = cv2.findContours(segment, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		max_contour = max(contours, key=cv2.contourArea)
		epsilon = 0.01*cv2.arcLength(max_contour, True)
		max_contour = cv2.approxPolyDP(max_contour, epsilon, True)

		hull = cv2.convexHull(max_contour, returnPoints=False)
		defects = cv2.convexityDefects(max_contour, hull)

		return (max_contour, defects)

	def _detect_num_fingers(self, contours, defects, outimg):
		if defects is None:
			return [outimg, 0]

		if len(defects) <= 2:
			return [outimg, 0]

		num_fingers = 1

		for i in range(defects.shape[0]):
			start_idx, end_idx, farthest_idx, _ = defects[i, 0]
			start = tuple(contours[start_idx][0])
			end = tuple(contours[end_idx][0])
			far = tuple(contours[farthest_idx][0])

			# cv2.line(outimg, start, end, [0, 255, 0], 2)

			if angle_rad(np.subtract(start, far), np.subtract(end, far)) < deg2rad(self.thresh_deg):
				num_fingers = num_fingers + 1
				
				cv2.circle(outimg, far, 5, [0, 255, 0], -1)
			# else:
				# cv2.circle(outimg, far, 5, [0, 0, 255], -1)

		return (outimg, min(5, num_fingers))

def angle_rad(v1, v2):
	return np.arctan2(np.linalg.norm(np.cross(v1, v2)), np.dot(v1, v2))

def deg2rad(angle_deg):
	return angle_deg/180.0*np.pi


