import cv2


def draw_center_frame(frame, center):
    cv2.circle(frame, center, 5, (255, 0, 0), 4)
    
