from .draw_square_frame import draw_square_frame
from .draw_aruco_id import draw_aruco_id
from .get_aruco_corner_and_center import get_corner_and_center
import cv2 
import numpy as np


def display_distance_marker(frame, marker_id, distance):
    """
    Displays the marker ID and its distance on the frame.

    Parameters:
        frame (numpy.ndarray): The image frame to display the text on.
        marker_id (int): The ID of the marker.
        distance (float): The distance to the marker in centimeters.
    """
    cv2.putText(
        frame,
        f"Marker: {marker_id}, Distance: {distance:.2f} cm",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 255, 255),
        3
    )


def estimatePoseAndTransformation(marker, camera_matrix, distortion_coeff, marker_length=0.1):
    """
    Estimates the pose of an ArUco marker and returns the transformation matrix along with its distance,
    rotation vector, and translation vector.

    Parameters:
        marker (numpy.ndarray): Marker corners.
        camera_matrix (numpy.ndarray): Camera intrinsic matrix.
        distortion_coeff (numpy.ndarray): Camera distortion coefficients.
        marker_length (float): The physical length of the marker in meters (default is 0.1).

    Returns:
        tuple: (transformation_matrix, distance, rvec, tvec)
            - transformation_matrix (numpy.ndarray): 4x4 transformation matrix.
            - distance (float): Distance to the marker in centimeters.
            - rvec (numpy.ndarray): Rotation vector.
            - tvec (numpy.ndarray): Translation vector.
    """
    # Estimate pose of the marker and get the transformation matrix
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker, marker_length, camera_matrix, distortion_coeff)
    R, _ = cv2.Rodrigues(rvec)
    print(f"This is the marker length{marker_length}")
    transformation_matrix = np.hstack((R, tvec[0].T))
    transformation_matrix = np.vstack((transformation_matrix, np.array([0, 0, 0, 1])))

    # Calculate the norm distance of the marker in centimeters
    distance = np.linalg.norm(tvec) * 100
    return transformation_matrix, distance, rvec, tvec


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
    try:
        print("Inside marker renderer")
        marker_coordinates = get_corner_and_center(marker)
        transformation_matrix, distance, rvec, tvec = estimatePoseAndTransformation(
            marker, camera_matrix, distortion_coefficient, marker_length
        )
        print("Success")
        draw_square_frame(frame, marker_coordinates)
        draw_aruco_id(frame, marker_coordinates, marker_id)
        display_distance_marker(frame,marker_id,distance)
        cv2.drawFrameAxes(frame, camera_matrix, distortion_coefficient, rvec, tvec, marker_length)
     
        return transformation_matrix
    except Exception as e:
        return f"There was an error computing aruco tf matrix {e}"
    