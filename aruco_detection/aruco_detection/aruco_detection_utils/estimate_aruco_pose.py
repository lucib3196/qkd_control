
def estimate_marker_pose_single(frame, marker, camera_matrix, distortion_coeff, marker_id, marker_length=0.1):
    """Estimate the pose of an ArUco marker and display its distance on the frame.

    Args:
        frame (numpy.ndarray): The image frame where the marker is detected.
        marker (numpy.ndarray): The detected marker corners.
        camera_matrix (numpy.ndarray): Camera matrix for intrinsic parameters.
        distortion_coeff (numpy.ndarray): Distortion coefficients of the camera.
        marker_id (int): ID of the marker being processed.
        marker_length (float, optional): Length of the marker's side in meters. Defaults to 0.1.

    Returns:
        None: The function modifies the input frame to display marker ID and distance.
    """

    # Estimate the pose of the marker
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker, marker_length, camera_matrix, distortion_coeff)

    # Calculate the distance to the marker in centimeters
    distance = np.linalg.norm(tvec) * 100  # Convert to cm

    # Display the marker ID and distance on the frame
    cv2.putText(
        frame,
        f"Marker: {marker_id}, Distance: {distance:.2f} cm",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 255, 255),
        3
    )
    return distance