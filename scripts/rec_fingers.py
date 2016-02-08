#!/usr/bin/env python

import cv2
import cv2.cv as cv
import numpy as np 

class RecognizeNumFingers:
	def __init__(self):
		self.abs_depth_dev = 14

	def find(self, img):
		self.height, self.width = img.shape[:2]

		armImg = self._extract_arm(img)		

		return armImg

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