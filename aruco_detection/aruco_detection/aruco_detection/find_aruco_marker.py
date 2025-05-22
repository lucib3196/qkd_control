import cv2.aruco as aruco

def find_aruco_marker(frame, aruco_dict, parameters):
    """
    Detects ArUco markers in a given frame.

    Parameters:
        frame (numpy.ndarray): The input frame (grayscale or color image) in which to detect ArUco markers.
        aruco_dict (cv2.aruco.Dictionary): The ArUco dictionary to use for marker detection.
        parameters (cv2.aruco.DetectorParameters): Detection parameters for the ArUco detector.

    Returns:
        list: A list of tuples, where each tuple contains the detected marker's corners and its ID.
              Example: [((corner1, corner2, corner3, corner4), id), ...]
    """
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    markers, ids, _ = detector.detectMarkers(frame)

    marker_arr = []
    if ids is not None:
        for marker_corner, marker_id in zip(markers, ids):
            marker_arr.append((marker_corner, marker_id))

    return marker_arr