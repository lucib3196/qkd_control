import cv2
import cv2.aruco as aruco
import numpy as np

from .draw_aruco_id import *
from .draw_square_frame import *
from .estimate_aruco_pose import *
from .find_aruco_marker import *
from .get_aruco_corner_and_center import *

def track_and_render_marker(frame, marker, marker_id, camera_matrix, distortion_coefficient, marker_length):
    """
    Tracks the marker and renders its square frame, axis, and ID on the frame.

    This function estimates the pose of the marker, draws a square frame around it,
    annotates its ID, and renders the coordinate axes based on the estimated pose.
    It serves as the main function for marker tracking in the project.

    Parameters:
        frame (numpy.ndarray): The frame to draw on.
        marker (numpy.ndarray): The detected marker's corners.
        marker_id (int): The ID of the marker.
        camera_matrix (numpy.ndarray): The camera's intrinsic matrix.
        distortion_coefficient (numpy.ndarray): The camera's distortion coefficients.
        marker_length (float): The physical length of the marker in meters.
    """
    marker_coordinates = get_corner_and_center(marker)
    transformation_matrix, distance, rvec, tvec = estimatePoseAndTransformation(
        marker, camera_matrix, distortion_coefficient, marker_length
    )
    draw_square_frame(frame, marker_coordinates)
    draw_id(frame, marker_coordinates, marker_id)
    display_distance_marker(frame,marker_id,distance)
    cv2.drawFrameAxes(frame, camera_matrix, distortion_coefficient, rvec, tvec, marker_length)
    return transformation_matrix