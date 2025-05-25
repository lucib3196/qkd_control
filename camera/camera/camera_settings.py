"""
Camera Configuration Settings

These settings are shared by both the publishing and receiving camera nodes.
They define the calibration parameters, video stream settings, and marker detection configuration.
"""

from pydantic import BaseModel, Field
from typing import Literal
from .utils.camera_calibration_utils import CalibratedCamera

# -----------------------------------------------------------------------------
# 🔧 Camera Calibration (Intrinsic Parameters)
# -----------------------------------------------------------------------------

CAMERA_MATRIX_PATH = "qkd_control/camera/camera/utils/calibrated/imx708/cameraMatrix.pkl"
CAMERA_DISTORTION_PATH = "qkd_control/camera/camera/utils/calibrated/imx708/dist.pkl"

calibrated_camera = CalibratedCamera(
    cam_mat_path=CAMERA_MATRIX_PATH,
    cam_dist_path=CAMERA_DISTORTION_PATH,
    frame_width=1536,
    frame_height=864
)

# -----------------------------------------------------------------------------
# 🎥 Camera Stream Configuration
# Used for PiCamera2 video config and frame publishing settings.
# -----------------------------------------------------------------------------

camera_config = {
    "brightness": 0.0,
    "contrast": 0.8,
    "saturation": 1.3,
    "framerate": 30
}

# -----------------------------------------------------------------------------
# 🎯 ArUco Marker Detection Parameters
# Used for pose estimation and optional offset tracking.
# -----------------------------------------------------------------------------

class ArucoParameters(BaseModel):
    aruco_dict_type: Literal[
        "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
        "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
        "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
        "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000"
    ] = Field(
        default="DICT_5X5_1000",
        description="Type of ArUco dictionary used for marker detection."
    )

    marker_length: float = Field(
        default=0.033,
        description="Length of the ArUco marker's side in meters."
    )

    marker_id: int = Field(
        default=5,
        description="ID of the ArUco marker to track."
    )

    track_point: bool = Field(
        default=False,
        description="Whether to track an offset point on the marker."
    )

    marker_point: list[float] = Field(
        default_factory=lambda: [0.1, 0, 0, 1],
        description="Offset of the point to track in marker coordinates (meters)."
    )

