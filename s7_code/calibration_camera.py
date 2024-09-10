import cv2 as cv
import os
import numpy as np

# Checker board size
CHESS_BOARD_DIM = (8,6 )

#size  Square 
SQUARE_SIZE = 22  # millimeters

# criteria EPS and ITER
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

calib_data_path = "python/calib_data"
CHECK_DIR = os.path.isdir(calib_data_path)

if not CHECK_DIR:
    os.makedirs(calib_data_path)
    print('"{calib_data_path}" Directory is created')
else:
    print('"{calib_data_path}" Directory already Exists.')

# prepare object points
obj_3D = np.zeros((CHESS_BOARD_DIM[0] * CHESS_BOARD_DIM[1], 3), np.float32)
obj_3D[:, :2] = np.mgrid[0:CHESS_BOARD_DIM[0], 0:CHESS_BOARD_DIM[1]].T.reshape(-1, 2)
obj_3D *= SQUARE_SIZE

# object points image points 
obj_points_3D = []  # 3d point in real world space
img_points_2D = []  # 2d points in image plane.


cap = cv.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    grayScale = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(frame, CHESS_BOARD_DIM, None)
    if ret:
        obj_points_3D.append(obj_3D)
        corners2 = cv.cornerSubPix(grayScale, corners, (3, 3), (-1, -1), criteria)
        
        img_points_2D.append(corners2)

        img = cv.drawChessboardCorners(frame, CHESS_BOARD_DIM, corners2, ret)

    cv.imshow("Calibration", frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()


# Convert lists to numpy arrays
obj_points_3D = np.array(obj_points_3D)
img_points_2D = np.array(img_points_2D)

print("j'attends")
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
    obj_points_3D, img_points_2D, grayScale.shape[::-1], None, None
)
print("Calibrated")

print("Saving calibration data to npz file")
np.savez(
    "{}{}".format(calib_data_path, "/CameraCalibrationData.npz"),
    camMatrix=mtx,
    distCoef=dist,
    rVector=rvecs,
    tVector=tvecs,
)


print("Calibration data saved successfully")

