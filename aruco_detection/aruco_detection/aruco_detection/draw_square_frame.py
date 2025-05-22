
def draw_square_frame(image_frame, coordinates):
    try:
        [top_left, top_right, bottom_right, bottom_left, center] = coordinates
        # Draw lines connecting the marker corners
        cv2.line(image_frame, top_left, top_right, (0, 255, 0), 2)
        cv2.line(image_frame, top_right, bottom_right, (0, 255, 0), 2)
        cv2.line(image_frame, bottom_right, bottom_left, (0, 255, 0), 2)
        cv2.line(image_frame, bottom_left, top_left, (0, 255, 0), 2)
        
        cv2.circle(image_frame, center, 5, (255, 0, 255), cv2.FILLED) # Center dot 
    except Exception as e:
        raise ("Error Drawing Square Frame  For Marker{e}")
    