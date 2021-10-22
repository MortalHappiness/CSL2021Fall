import cv2
import numpy as np
import matplotlib.pyplot as plt

# Choose webcam: 0, 1, ...
# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(1)

text = ""

pts = np.zeros(shape=(0, 1, 2), dtype=np.int32)
LEAVE_TIMEOUT = 50
ENTER_TIMEOUT = 30
DISPLAY_TIMEOUT = 50
leave_timeout = 0
enter_timeout = 0
display_timeout = 0

def predict(img, points):
	return 8

while(True):
	# Capture one frame from the camera
	ret, frame = cap.read()
	frame = cv2.flip(frame, 1)

	# Split RGB channels
	B, G, R = cv2.split(frame)

	# Perform thresholding to each channel
	_, R = cv2.threshold(R, 120, 255, cv2.THRESH_BINARY)
	# _, G = cv2.threshold(G, 100, 255, cv2.THRESH_BINARY_INV)
	_, B = cv2.threshold(B, 90, 255, cv2.THRESH_BINARY_INV)

	# Get final result using bitwise operation
	Display = cv2.bitwise_and(R, B, mask=None)

	# Blur
	Display = cv2.blur(Display, (15, 15))
	_, Display = cv2.threshold(Display, 90, 255, cv2.THRESH_BINARY)

	# Find and draw contours
	contour, hierarchy = cv2.findContours(Display, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	## for debug
	Display = cv2.cvtColor(Display, cv2.COLOR_GRAY2BGR)
	cv2.drawContours(Display, contour, -1, (0, 0, 255))

	# Iterate through each contours, check area and find center
	x, y = None, None
	for cnt in contour: 
		area = cv2.contourArea(cnt)
		M = cv2.moments(cnt)
		if M['m00'] != 0 and area > 7000:
			x, y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

	if x is not None and y is not None:
		leave_timeout = LEAVE_TIMEOUT
		display_timeout = 0
		enter_timeout += 1
		cv2.putText(Display, f"{area}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255))
		cv2.circle(Display, (x, y), radius=10, color=(0, 0, 255), thickness=-1)

		if enter_timeout >= ENTER_TIMEOUT:
			if pts.shape[0] == 0 or not (pts[-1, 0, 0] == x and pts[-1, 0, 1] == y):
				pts = np.concatenate((pts, np.array([x, y]).reshape((-1, 1, 2))))
	else:
		enter_timeout = 0

	if leave_timeout > 0:
		leave_timeout -= 1
		cv2.polylines(Display, [pts], False, (255, 255, 0), 4)
	elif pts.shape != (0, 1, 2):
		# Predict number
		text = str(predict(Display, pts))
		pts = np.zeros(shape=(0, 1, 2), dtype=np.int32)
		display_timeout = DISPLAY_TIMEOUT
	
	if display_timeout > 0:
		display_timeout -= 1
		cv2.putText(Display, text, (200, 400), cv2.FONT_HERSHEY_SIMPLEX, 15, (0, 255, 255), 20)

	# Show image
	debug = False
	if debug:
		Display = cv2.hconcat([frame, Display, np.zeros_like(frame)])
		Display = cv2.vconcat([Display, cv2.hconcat([cv2.cvtColor(x, cv2.COLOR_GRAY2BGR) for x in [R, G, B]])])
		zeros = np.zeros_like(R)
		R = cv2.merge([zeros, zeros, R])
		G = cv2.merge([zeros, G, zeros])
		B = cv2.merge([B, zeros, zeros])
		Display = cv2.vconcat([Display, cv2.hconcat([R, G, B])])
		resize_scale = 0.5
		scale_percent = 60
		width = int(Display.shape[1] * scale_percent / 100)
		height = int(Display.shape[0] * scale_percent / 100)
		dim = (width, height)
		Display = cv2.resize(Display, dim)

	cv2.imshow('frame', Display)

	# Press q to quit
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release camera
cap.release()

# Close all OpenCV windows
cv2.destroyAllWindows()