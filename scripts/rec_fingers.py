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

		armImg = self._extract_arm(img)	

		(contours, defects) = self._find_hull_defects(armImg)

		outImg = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

		(outImg, num_fingers) = self._detect_num_fingers(contours, defects, outImg)

		return (outImg, num_fingers)

	def _extract_arm(self, img):
		# find center region of image frame (assume center region is 21 x 21 px)
		center_half = 10 # (=(21-1)/2)	
		center = img[self.height/2 - center_half : self.height/2 + center_half, self.width/2 - center_half : self.width/2 + center_half]

		# determine median depth value
		median_val = np.median(center)

		'''mask the image such that all pixels whose depth values
		lie within a particular range are gray and the rest are black
		'''

		img = np.where(abs(img-median_val) <= self.abs_depth_dev, 128, 0).astype(np.uint8)

		# Apply morphology operation to fill small holes in the image
		kernel = np.ones((5,5), np.uint8)
		img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

		# Find connected regions (to hand) to remove background objects
		# Use floodfill with a small image area (7 x 7 px) that is set gray color value
		kernel2 = 3
		img[self.height/2-kernel2:self.height/2+kernel2, self.width/2-kernel2:self.width/2+kernel2] = 128
		
		# a black mask to mask the 'non-connected' components black
		mask = np.zeros((self.height + 2, self.width + 2), np.uint8)
		floodImg = img.copy()

		# Use floodFill function to paint the connected regions white 
		cv2.floodFill(floodImg, mask, (self.width/2, self.height/2), 255, flags=(4 | 255 << 8))
		
		# apply a binary threshold to show only connected hand region
		ret, floodedImg = cv2.threshold(floodImg, 129, 255, cv2.THRESH_BINARY)

		return floodedImg

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

			cv2.line(outimg, start, end, [0, 255, 0], 2)

			if angle_rad(np.subtract(start, far), np.subtract(end, far)) < deg2rad(self.thresh_deg):
				num_fingers = num_fingers + 1
				
				cv2.circle(outimg, far, 5, [0, 255, 0], -1)
			else:
				cv2.circle(outimg, far, 5, [0, 0, 255], -1)

		return (outimg, min(5, num_fingers))

def angle_rad(v1, v2):
	return np.arctan2(np.linalg.norm(np.cross(v1, v2)), np.dot(v1, v2))

def deg2rad(angle_deg):
	return angle_deg/180.0*np.pi