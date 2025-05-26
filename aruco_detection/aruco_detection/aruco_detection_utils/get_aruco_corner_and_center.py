
def get_corner_and_center(marker_corner):
    """
    Calculates the corners and center of an ArUco marker.

    Parameters:
        marker_corner (numpy.ndarray): A 4x2 array containing the coordinates of the marker's corners.
                                       Example: [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]

    Returns:
        list: A list containing the four corners (top-left, top-right, bottom-right, bottom-left) as tuples of integers,
              and the center point as a tuple.
              Example: [(top_left), (top_right), (bottom_right), (bottom_left), (center)]
    """
    corners_abcd = marker_corner.reshape((4, 2))
    top_left, top_right, bottom_right, bottom_left = corners_abcd

    top_left = (int(top_left[0]), int(top_left[1]))
    top_right = (int(top_right[0]), int(top_right[1]))
    bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
    bottom_left = (int(bottom_left[0]), int(bottom_left[1]))

    center_x = (top_left[0] + top_right[0] + bottom_right[0] + bottom_left[0]) // 4
    center_y = (top_left[1] + top_right[1] + bottom_right[1] + bottom_left[1]) // 4
    center = (center_x, center_y)

    return [top_left, top_right, bottom_right, bottom_left, center]