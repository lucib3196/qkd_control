
def draw_aruco_id(frame, coor, marker_id):
    """
    Draws the marker ID at the top-left corner of the marker on the frame.

    Parameters:
        frame (numpy.ndarray): The frame/image to draw on.
        coor (list): A list of corner coordinates, where coor[0] is the top-left corner.
        marker_id (int): The ID to draw.

    Returns:
        numpy.ndarray: The frame with the marker ID drawn.
    """
    font = cv2.FONT_HERSHEY_PLAIN
    font_scale = 1
    color = (0, 255, 0)
    thickness = 2

    top_right = coor[1]
    cv2.putText(frame, str(marker_id), top_right, font, font_scale, color, thickness)
    return frame
