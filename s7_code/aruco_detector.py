# -*- coding: utf-8 -*-

import cv2 as cv
from cv2 import aruco
import numpy as np

calib_data_path = "python/calib_data/CameraCalibrationData.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]

MARKER_SIZE = 14.8  # centimeters

marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)

param_markers = aruco.DetectorParameters_create()

cap = cv.VideoCapture(0)
while True:

    _ ,frame = cap.read()   
    # Détecter les marqueurs
    marker_corners, marker_IDs, _ = aruco.detectMarkers(
            frame, marker_dict, parameters=param_markers
    )

    if marker_corners:
        # Estimer la pose de chaque marqueur
        rvecs, tvecs = aruco.estimatePoseSingleMarkers(
              marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = tuple(corners[0])
            top_left = tuple(corners[1])
            bottom_right = tuple(corners[2])
            bottom_left = tuple(corners[3])

            distance = np.linalg.norm(tvecs[i][0])
   
            # Draw 
            aruco.drawAxis(frame, cam_mat, dist_coef, rvecs[i], tvecs[i], 4)

            # Display 
            cv.putText(
                frame,
                "id: {} Dist: {}".format(ids[0], round(distance, 2)),
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )
            cv.putText(
                frame,
               "x: {} y: {}".format(round(tvecs[i][0][0], 1), round(tvecs[i][0][1], 1)),
                bottom_right,
                cv.FONT_HERSHEY_PLAIN,
                1.0,
                (0, 0, 255),
                2,
                cv.LINE_AA,
        )
        print(tvecs)

    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
            break

cap.release()

cv.destroyAllWindows()

