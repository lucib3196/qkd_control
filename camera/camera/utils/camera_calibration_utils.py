"""
camera_calibration_utils.py

Utility module for managing camera calibration using OpenCV. Provides a
CalibratedCamera class that loads intrinsic calibration parameters and
performs undistortion on captured frames.

Typical use case:
    camera = CalibratedCamera("camera_matrix.pkl", "camera_dist.pkl", 480, 640)
    undistorted = camera.undistort_frame(frame)

Author: [Your Name]
"""

import os
import cv2
import pickle
import logging
import numpy as np
from dataclasses import dataclass

# Optional: Configure default logging
logging.basicConfig(level=logging.INFO)

@dataclass
class CalibratedCamera:
    """
    Represents a camera that has been calibrated using intrinsic parameters.
    
    Attributes:
        cam_mat_path (str): Path to the pickled camera matrix file.
        cam_dist_path (str): Path to the pickled distortion coefficients file.
        frame_height (int): Height of the frames expected from the camera.
        frame_width (int): Width of the frames expected from the camera.
    """
    cam_mat_path: str
    cam_dist_path: str
    frame_height: int
    frame_width: int

    def __post_init__(self):
        """Post-initialization hook to load calibration data and prepare for undistortion."""
        self.load_calibration_parameters()
        self.calibrate_camera()

    def load_calibration_parameters(self):
        """
        Load the camera matrix and distortion coefficients from disk.
        The paths are relative to the current working directory.
        """
        camera_matrix_path = os.path.join(os.getcwd(), self.cam_mat_path)
        camera_distortion_path = os.path.join(os.getcwd(), self.cam_dist_path)

        try:
            with open(camera_matrix_path, "rb") as file:
                self.camera_matrix = pickle.load(file)
            with open(camera_distortion_path, "rb") as file:
                self.camera_dist = pickle.load(file)
            logging.info("Camera calibration parameters loaded successfully.")
        except Exception as e:
            logging.error(f"Error loading camera calibration files: {e}")
            raise FileNotFoundError("Calibration file not found or corrupted.") from e

    def calibrate_camera(self):
        """
        Compute an optimal new camera matrix and region of interest for undistortion.
        """
        self.new_cam_mtx, self.roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.camera_dist,
            (self.frame_width, self.frame_height), 1,
            (self.frame_width, self.frame_height)
        )

    def undistort_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        Undistort the given image using the loaded calibration parameters.

        Args:
            frame (np.ndarray): A distorted image frame captured from the camera.

        Returns:
            np.ndarray: The undistorted image cropped to the valid ROI.
        """
        undistorted_frame = cv2.undistort(
            frame, self.camera_matrix, self.camera_dist, None, self.new_cam_mtx
        )
        x, y, w, h = self.roi
        return undistorted_frame[y:y + h, x:x + w]
