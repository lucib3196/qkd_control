int32 id
# Unique identifier for the detected ArUco marker.
# This ID corresponds to the specific marker in the predefined dictionary,
# allowing differentiation between multiple markers.

geometry_msgs/Pose pose
# Represents the pose (position and orientation) of the marker relative to the camera frame.
# - position: geometry_msgs/Point with x, y, z coordinates in meters.
# - orientation: geometry_msgs/Quaternion representing the rotation in space.

builtin_interfaces/Time detection_time
# Timestamp indicating when the marker was detected.
# Useful for synchronizing with other sensor data and for temporal analysis.

float32 size
# Physical size of the marker's side in meters.
# Essential for accurate pose estimation and for scaling in visualizations.

string dictionary_type
# Specifies the ArUco dictionary used for detection (e.g., "DICT_4X4_50").
# Ensures correct interpretation of marker IDs and facilitates compatibility checks.
