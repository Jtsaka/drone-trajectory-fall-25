from dataclasses import dataclass
"""Data models for the camera and user specification."""

@dataclass
class DatasetSpec:
    """
    Data model for specifications of an image dataset.
    """

    overlap: float #ratio (in 0 to 1) of scene shared between two consecutive images.
    sidelap: float #ratio (in 0 to 1) of scene shared between two images in adjacent rows.
    height: float #the height of the scan above the ground (in meters).
    scan_dimension_x: int #horizontal size of the rectangle to be scanned (in meters).
    scan_dimension_y: int #vertical size of the rectangle to be scanned (in meters).
    exposure_time_ms: int #exposure time for each image (in milliseconds).

@dataclass
class Camera:
    """
    Data model for a simple pinhole camera.

    References:
    - https://github.com/colmap/colmap/blob/3f75f71310fdec803ab06be84a16cee5032d8e0d/src/colmap/sensor/models.h#L220
    - https://en.wikipedia.org/wiki/Pinhole_camera_model
    """

    fx: float #focal length along x axis (in pixels)
    fy: float #focal length along y axis (in pixels)
    cx: float #optical center of the image along the x axis (in pixels)
    cy: float #optical center of the image along the y axis (in pixels)
    sensor_size_x_mm: float #Size of the sensor along the x axis (in mm)
    sensor_size_y_mm: float #Size of the sensor along the y axis (in mm)
    image_size_x_px: int #Number of pixels in the image along the x axis
    image_size_y_px: int #Number of pixels in the image along the y axis


class Waypoint:
    """
    Waypoints are positions where the drone should fly to and capture a photo.
    """

    pass
