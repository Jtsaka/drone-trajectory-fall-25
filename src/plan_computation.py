import typing as T
import math

import numpy as np

from src.data_model import Camera, DatasetSpec, Waypoint
from src.camera_utils import (
    compute_image_footprint_on_surface,
    compute_ground_sampling_distance,
)


def compute_distance_between_images(
    camera: Camera, dataset_spec: DatasetSpec
) -> np.ndarray:
    """Compute the distance between images in the horizontal and vertical directions for specified overlap and sidelap.

    Args:
        camera: Camera model used for image capture.
        dataset_spec: user specification for the dataset.

    Returns:
        The horizontal and vertical distance between images (as a 2-element array).
    """
    
    footprint_x, footprint_y = compute_image_footprint_on_surface(camera, dataset_spec.height)
    horizontal_distance = footprint_x - (dataset_spec.sidelap * footprint_x)
    vertical_distance = footprint_y - (dataset_spec.overlap * footprint_y)

    return np.array([horizontal_distance, vertical_distance])


def compute_speed_during_photo_capture(
    camera: Camera, dataset_spec: DatasetSpec, allowed_movement_px: float = 1
) -> float:
    """Compute the speed of drone during an active photo capture to prevent more than 1px of motion blur.

    Args:
        camera: Camera model used for image capture.
        dataset_spec: user specification for the dataset.
        allowed_movement_px: The maximum allowed movement in pixels. Defaults to 1 px.

    Returns:
        The speed at which the drone should move during photo capture.
    """
    exposure_time_s = dataset_spec.exposure_time_ms / 1000 #convert from milliseconds to seconds
    drone_speed_mps = (allowed_movement_px * compute_ground_sampling_distance(camera, dataset_spec.height))/exposure_time_s

    return drone_speed_mps


def generate_photo_plan_on_grid(
    camera: Camera, dataset_spec: DatasetSpec
) -> T.List[Waypoint]:
    """Generate the complete photo plan as a list of waypoints in a lawn-mower pattern.

    Args:
        camera: Camera model used for image capture.
        dataset_spec: user specification for the dataset.

    Returns:
        Scan plan as a list of waypoints.

    """
    raise NotImplementedError()