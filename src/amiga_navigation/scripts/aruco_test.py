import cv2
import numpy as np

# Check OpenCV version for aruco module
if cv2.__version__.startswith('3.'):
    aruco = cv2.aruco
else:
    aruco = cv2.aruco

# Parameters for the ArUco board
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
markers_x = 5  # Number of markers along the X-axis
markers_y = 7  # Number of markers along the Y-axis
marker_length = 0.04  # Marker side length in meters (e.g., 4 cm)
marker_separation = 0.01  # Separation between markers in meters (e.g., 1 cm)

# Create the grid board
board = aruco.GridBoard(
    (markers_x, markers_y),
    marker_length, marker_separation,
    aruco_dict)

# Modify the object points to make Z-axis upwards
# By default, the board is in the X-Y plane. We'll rotate it to lie in the X-Z plane.
print()

# Get the original object points
obj_points = board.getObjPoints()  # List of numpy arrays
# Rotate the points around the X-axis to make Z-axis upwards
rotation_matrix = np.array([
    [1, 0, 0],
    [0, 0, 1],  # Swap Y and Z
    [0, -1, 0]  # Invert the new Y-axis
])

# Apply the rotation to each marker's corners
for i in range(len(obj_points)):
    for j in range(len(obj_points[i])):
        obj_points[i][j] = obj_points[i][j].dot(rotation_matrix)

# Update the board's object points
board.objPoints = obj_points

# Now, when you detect the board and estimate its pose, the Z-axis will point upwards.

# Example of detecting the board and estimating pose
# Assuming you have a camera matrix `camera_matrix` and distortion coefficients `dist_coeffs`
# Also assuming you have an input image `image` from your camera

# image = cv2.imread('path_to_image.jpg')  # Load your image

# Convert to grayscale
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect markers in the image
# corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)

# If markers are detected
# if ids is not None:
#     # Refine detection
#     corners, ids, rejected, recovered_ids = aruco.refineDetectedMarkers(
#         image=gray,
#         board=board,
#         detectedCorners=corners,
#         detectedIds=ids,
#         rejectedCorners=rejected,
#         cameraMatrix=camera_matrix,
#         distCoeffs=dist_coeffs)

#     # Estimate pose of the board
#     ret, rvec, tvec = aruco.estimatePoseBoard(
#         corners, ids, board, camera_matrix, dist_coeffs, None, None)

#     if ret:
#         # Draw the board frame axes on the image
#         aruco.drawAxis(image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
#         cv2.imshow('Pose Estimation', image)
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()
# else:
#     print("No markers detected.")

# Note: You need to calibrate your camera to get `camera_matrix` and `dist_coeffs`
