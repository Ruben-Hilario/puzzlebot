#This is not a ros2 node, its just a method to calibrate the camera using opencv
#The following code relies on a set of previously captured images of a chessboard pattern, store them where is more convenient
import os
import cv2 as cv
import numpy as np
from glob import glob as glob

c_imgs = glob('/home/brad/.temp/residuals/puzzlebot3/*png')

def cameraCalibration():
	#parameter def
	criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
	objp = np.zeros((5*7,3), np.float32)
	objp[:,:2] = np.mgrid[0:7,0:5].T.reshape(-1,2)
	objP = []
	imgP = []
	try:
		for i in c_imgs:
			img = cv.imread(i)
			gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
			ret, corners = cv.findChessboardCorners(gray,(7,5),None)
			if ret == True:
				objP.append(objp)
				corners2=cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
				imgP.append(corners2)
			print(f"procesed image {i}")
	except Exception as e:
		print(f"{e}")
		exit
	return cv.calibrateCamera(objP, imgP, gray.shape[::-1], None, None)

def main():
	rms, k, dist, rvecs, tvecs =  cameraCalibration() 
	print("Calibration Matrix", k)
	print("Distortion Coefficients", dist)

	

if __name__ == "__main__":
	main()