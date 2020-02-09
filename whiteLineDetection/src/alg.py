#!/usr/bin/env python3
import numpy as np
import cv2

import os
import glob

import matplotlib.pyplot as plt

import time

import pyzed.sl as sl
import rospy

class Detect:
		
	def __init__(self):
		self.zed = sl.Camera() 
		#GET IMAGE FROM ZED
		init_params = sl.InitParameters()
		init_params.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720
		init_params.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_PERFORMANCE
		init_params.coordinate_system = sl.COORDINATE_SYSTEM.COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD
		init_params.coordinate_units = sl.UNIT.UNIT_METER

		err = self.zed.open(init_params)
		if (err != sl.ERROR_CODE.SUCCESS):
				print("HAHA FUCK YOU GUYS!!!\n\tSincerely, the ZED team.")
				exit(-1)


	def seperate_hls(self,rgb_img):
		hls = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HLS)
		h = hls[:,:,0]
		l = hls[:,:,1]
		s = hls[:,:,2]
		return h, l, s

	def seperate_lab(self,rgb_img):
		lab_u = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2Lab)
		lab = lab_u.get()
		l = lab[:,:,0]
		a = lab[:,:,1]
		b = lab[:,:,2]
		return l, a, b

	def seperate_luv(self,rgb_img):
		luv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2Luv)
		l = luv[:,:,0]
		u = luv[:,:,1]
		v = luv[:,:,2]
		return l, u, v

	def binary_threshold_lab_luv(self,rgb_img,rgb_img_g, bthresh, lthresh):
		l, a, b = self.seperate_lab(rgb_img_g)
		l2, u, v = self.seperate_luv(rgb_img)
		first = time.time()
		binary = np.zeros_like(l)
		binary[
			((b > bthresh[0]) & (b <= bthresh[1])) |
			((l2 > lthresh[0]) & (l2 <= lthresh[1]))
		] = 1
		return binary

	def gradient_threshold(self,channel, thresh):
		# Take the derivative in x
		sobelx = cv2.Sobel(channel, cv2.CV_64F, 1, 0)
		# Absolute x derivative to accentuate lines away from horizontal
		abs_sobelx = np.absolute(sobelx)
		scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
		# Threshold gradient channel
		sxbinary = np.zeros_like(scaled_sobel)
		sxbinary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
		return sxbinary
			
				
	def close(self):
		self.zed.close()

	  
	def main(self,callback_data):
		first = time.time()
	
		image = sl.Mat()
		
		if (self.zed.grab(sl.RuntimeParameters()) == sl.ERROR_CODE.SUCCESS):
				# A new image is available if grab() returns SUCCESS
				self.zed.retrieve_image(image, sl.VIEW.VIEW_LEFT)
		

		#TEST_IMAGE = "assets/Im3.png"
		lane_test_image_cpu = image.get_data()
		lane_test_image = cv2.UMat(lane_test_image_cpu)
		lane_test_image = cv2.cvtColor(lane_test_image,cv2.COLOR_BGR2RGB)
		#plt.imshow(lane_test_image_cpu)

	#	cv2.imshow("yo",lane_test_image_cpu)
	#	cv2.waitKey(1000)
		


		GRADIENT_THRESH = (20,100)
		L_CHANNEL_THRESH = (130,255)
		B_CHANNEL_THRESH = (170,210)

		s_binary = self.binary_threshold_lab_luv(lane_test_image_cpu,lane_test_image,B_CHANNEL_THRESH,L_CHANNEL_THRESH)
		h , l, s = self.seperate_hls(lane_test_image_cpu)
		sxbinary = self.gradient_threshold(s,GRADIENT_THRESH)
		color_binary = np.dstack((sxbinary,s_binary,np.zeros_like(sxbinary))) * 255
		#plt.imshow(color_binary)
		IMG_SIZE = lane_test_image_cpu.shape[::-1][1:]
		OFFSET = 300

		PRES_SRC_PNTS = np.float32([
			(450, 310), # Top-left corner
			(200, 720), # Bottom-left corner
			(820, 720), # Bottom-right corner
			(730, 310) # Top-right corner
		])

		PRES_DST_PNTS = np.float32([
		[OFFSET, 0], 
		[OFFSET, IMG_SIZE[1]],
		[IMG_SIZE[0]-OFFSET, IMG_SIZE[1]], 
		[IMG_SIZE[0]-OFFSET, 0]
		])
		
		lane_test_image_cp = cv2.UMat.get(lane_test_image)
		#plt.imshow(cv2.polylines(lane_test_image_cp,np.int32([PRES_SRC_PNTS]),True,(255,0,0),3));

		M = cv2.getPerspectiveTransform(PRES_SRC_PNTS,PRES_DST_PNTS)
		warped = cv2.warpPerspective(lane_test_image,M,IMG_SIZE,flags=cv2.INTER_LINEAR)

		
		warped_cp = cv2.UMat.get(warped)
		warped_poly = cv2.polylines(warped_cp, np.int32([PRES_DST_PNTS]), True, (255,0,0), 3)
		#plt.imshow(warped_poly)
		
		N_WINDOWS = 10
		MARGIN = 100
		RECENTER_MINPIX = 50

		YM_PER_PIX = 30/720
		XM_PER_PIXEL = 3.7/700
		
		binary_warped = cv2.warpPerspective(s_binary, M, IMG_SIZE, flags=cv2.INTER_LINEAR)

		histogram = np.sum(binary_warped[int(binary_warped.shape[0]/2):,:],axis=0)

	#	plt.plot(histogram)

	#	plt.imshow(binary_warped)

		leftx_base, rightx_base = self.histo_peak(histogram)
		left_lane_inds,right_lane_inds,nonzerox,nonzeroy, out_img = self.get_lane_indices_sliding_windows( binary_warped,leftx_base,rightx_base,N_WINDOWS,MARGIN,RECENTER_MINPIX)

		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds]
		rightx = nonzerox[right_lane_inds]
		righty = nonzeroy[right_lane_inds]

		left_fit = np.polyfit(lefty,leftx,2)
		right_fit = np.polyfit(righty,rightx,2)

		ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
		left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
		right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

		out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
		out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

	#	plt.imshow(warped_cp)

	#	plt.imshow(out_img)

		left_lane_inds,right_lane_inds, ploty, left_fitx,right_fitx = self.get_lane_indices_from_prev_window( binary_warped,left_fit,right_fit,MARGIN)

		out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
		window_img = np.zeros_like(out_img)

		# Color in left and right line pixels
		out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
		out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

		# Generate a polygon to illustrate the search window area
		# And recast the x and y points into usable format for cv2.fillPoly()
		left_line_pts = np.array([np.transpose(np.vstack([left_fitx-MARGIN, ploty]))])
		right_line_pts = np.array([np.transpose(np.vstack([right_fitx-MARGIN, ploty]))])

		cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
		cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
		result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)


		print(time.time() - first)
		

	#	f, axarr = plt.subplots(1,2)
	#	f.set_size_inches(18,5)
	#	axarr[0].imshow(binary_warped,cmap='gray')
	#	axarr[0].plot(left_fitx,ploty,color='red')
	#	axarr[0].plot(right_fitx,ploty,color='red')
	#	plt.show()

		

	def histo_peak(self,histo):
		"""Find left and right peaks of histogram"""
		midpoint = np.int(histo.shape[0]/2)
		leftx_base = np.argmax(histo[:midpoint])
		rightx_base = np.argmax(histo[midpoint:]) + midpoint
		return leftx_base, rightx_base

	def get_lane_indices_sliding_windows(self,binary_warped, leftx_base, rightx_base, n_windows, margin, recenter_minpix):
		"""Get lane line pixel indices by using sliding window technique"""
		# Create an output image to draw on and  visualize the result
		out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
		out_img = out_img.copy()
		# Set height of windows
		window_height = np.int(binary_warped.shape[0]/n_windows)

		# Identify the x and y positions of all nonzero pixels in the image
		nonzero = binary_warped.nonzero()
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])

		# Create empty lists to receive left and right lane pixel indices
		left_lane_inds = []
		right_lane_inds = []
		# Current positions to be updated for each window
		leftx_current = leftx_base
		rightx_current = rightx_base

		for window in range(n_windows):
			# Identify window boundaries in x and y (and right and left)
			win_y_low = binary_warped.shape[0] - (window + 1) * window_height
			win_y_high = binary_warped.shape[0] - window * window_height
			win_xleft_low = leftx_current - margin
			win_xleft_high = leftx_current + margin
			win_xright_low = rightx_current - margin
			win_xright_high = rightx_current + margin
			cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high), (0,255,0), 2)
			cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high), (0,255,0), 2)
			# Identify the nonzero pixels in x and y within the window
			good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
							  (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
			good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
							   (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
			# Append these indices to the lists
			left_lane_inds.append(good_left_inds)
			right_lane_inds.append(good_right_inds)
			# If you found > minpix pixels, recenter next window on their mean position
			if len(good_left_inds) > recenter_minpix:
				leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
			if len(good_right_inds) > recenter_minpix:        
				rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
			
		# Concatenate the arrays of indices
		left_lane_inds = np.concatenate(left_lane_inds)
		right_lane_inds = np.concatenate(right_lane_inds)
		return left_lane_inds, right_lane_inds, nonzerox, nonzeroy, out_img

	def get_lane_indices_from_prev_window(self,binary_warped, left_fit, right_fit, margin):
		"""Detect lane line by searching around detection of previous sliding window detection"""
		nonzero = binary_warped.nonzero()
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])

		left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + 
		left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) + 
		left_fit[1]*nonzeroy + left_fit[2] + margin))) 
		right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + 
		right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) + 
		right_fit[1]*nonzeroy + right_fit[2] + margin)))

		# Again, extract left and right line pixel positions
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds] 
		rightx = nonzerox[right_lane_inds]
		righty = nonzeroy[right_lane_inds]

		# Fit a second order polynomial to each
		left_fit = np.polyfit(lefty, leftx, 2)
		right_fit = np.polyfit(righty, rightx, 2)

		# Generate x and y values for plotting
		ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
		left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
		right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
		return left_lane_inds, right_lane_inds, ploty, left_fitx, right_fitx

	
if __name__ == "__main__":
	rospy.init_node('white_line', anonymous=True)
	detect = Detect()
	rospy.Timer(rospy.Duration(3), detect.main)
	# rospy.Subscriber('clock', Time, callback)
	rospy.spin()
	detect.close()
