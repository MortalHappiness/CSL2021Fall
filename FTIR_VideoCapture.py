import cv2
import numpy as np
import matplotlib.pyplot as plt

# Choose webcam: 0, 1, ...
# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(1)

while(True):
	# Capture one frame from the camera
	ret, frame = cap.read()

	# Split RGB channels
	B, G, R = cv2.split(frame)

	# Perform thresholding to each channel
	#_, R = cv2.threshold(R, 130, 255, cv2.THRESH_BINARY)
	#cv2.imshow('frame', R)
	_, G = cv2.threshold(G, 100, 255, cv2.THRESH_BINARY)
	_, B = cv2.threshold(B, 110, 255, cv2.THRESH_BINARY_INV)
	#cv2.imshow('frame', B)

	# Get final result using bitwise operation
	Resb = cv2.bitwise_and(G, B, mask=None)
	#cv2.imshow('frame', Result)

	# blur and then threshold again?
	#Resb = cv2.blur(Result, (5, 5))
	#_, Resb = cv2.threshold(blurred, 140, 255, cv2.THRESH_BINARY)

	# Find and draw contours
	contour, hierarchy = cv2.findContours(Resb, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	## for debug
	Display = cv2.cvtColor(Resb, cv2.COLOR_GRAY2BGR)
	cv2.drawContours(Display, contour, -1, (0, 0, 255))
	print([cv2.contourArea(cnt) for cnt in contour])

	# Iterate through each contours, check area and find center
	Display = cv2.flip(Display, 1)
	for cnt in contour : 
		area = cv2.contourArea(cnt)
		M = cv2.moments(cnt)
		if M['m00'] != 0 and area > 10000:
			x = M['m10']/M['m00']
			y = M['m01']/M['m00']
			pos = str(int(x))+str(int(y))
			cv2.putText(Display, pos, (int(x),int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
			Display = cv2.circle(Display, (int(x),int(y)), radius=10, color=(0, 0, 255), thickness=-1)

	# Show image
	#cv2.imshow('frame', frame)
	st = np.hstack((R, B, G, Resb))
	cv2.imshow('frame', st)
	#cv2.imshow('frame', Display)

	# Press q to quit
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release camera
cap.release()

# Close all OpenCV windows
cv2.destroyAllWindows()