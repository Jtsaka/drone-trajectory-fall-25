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
    
    drone_velocity = compute_speed_during_photo_capture(camera, dataset_spec, allowed_movement_px=1)

    dist_between_img_x, dist_between_img_y = compute_distance_between_images(camera, dataset_spec)
    num_point_x = math.ceil(dataset_spec.scan_dimension_x / dist_between_img_x) #ceiling rounds up
    num_point_y = math.ceil(dataset_spec.scan_dimension_y / dist_between_img_y)
    gaps_x = num_point_x - 1
    gaps_y = num_point_y - 1

    centers_x = []
    centers_y = []

    if num_point_x == 1: #if there's only 1 point available
        offset_x = dataset_spec.scan_dimension_x / 2
        centers_x = [offset_x]

    else:
        cent_dist_x = min(dist_between_img_x, dataset_spec.scan_dimension_x / gaps_x)
        offset_x = 0.5 * (dataset_spec.scan_dimension_x - cent_dist_x * gaps_x) 
        for index in range(num_point_x):
            centers_x.append(offset_x + index * cent_dist_x) #adding edge offset + iterating for each center

    if num_point_y == 1:
        offset_y = dataset_spec.scan_dimension_y / 2
        centers_y = [offset_y]

    else:
        cent_dist_y = min(dist_between_img_y, dataset_spec.scan_dimension_y / gaps_y)
        offset_y = 0.5 * (dataset_spec.scan_dimension_y - cent_dist_y * gaps_y)
        for index in range(num_point_y):
            centers_y.append(offset_y + index * cent_dist_y) 

    flight_plan = []
    z = dataset_spec.height
    for index, y in enumerate(centers_y): #enum to help iterate through y coords
        if index % 2 == 0:
            x_vals = centers_x
        else:
            x_vals = centers_x[::-1] #makes a copy, and then reverses when going up to next row
        for x in x_vals:
            flight_plan.append(Waypoint(x, y, z, speed_mps = drone_velocity, photo_trigger = True))

    return flight_plan

def waypoint_times(plan, acc, v_max):
    """Applies trapezoidal and calculates the time between each waypoint

    Args:
        plan: reads waypoints, used to iterate through each point for data
        acc: acceleration of drone
        v_max: maximum drone speed

    Returns:
        Time between each waypoint after trapezoidal profiling is applied

    """
    times = [0.0]
    time_total = 0.0
    
    for inc in range(len(plan) - 1):
        dist_x = plan[inc + 1].x - plan[inc].x
        dist_y = plan[inc + 1].y - plan[inc].y
        dist = math.hypot(dist_x, dist_y)
    
        v_blur = plan[inc].speed_mps
        trap_time, _, _, _ = apply_trapezoidal_prof(dist, acc, v_blur, v_max)
        time_total += trap_time

        times.append(time_total)
    return times

def apply_trapezoidal_prof(dist, acc, v_blur, v_max):
    """Calculates the trapezoidal profiling for the drone based on 4 inputs

    Args:
        dist: distance from one image point to another
        acc: acceleration of drone
        v_blur: calculated drone velocity for acceptable blur
        v_max: maximum drone speed

    Returns:
        Time between each waypoint after trapezoidal profiling is applied

    """
    s_acc = (v_max**2 - v_blur**2) / (2 * acc)
    if 2 * s_acc <= dist: #if a trapezoid
        t_acc = (v_max - v_blur) / acc
        t_cruise = (dist - 2 * s_acc) / v_max
        t_total = 2 * t_acc + t_cruise

        times = [0.0, t_acc, t_acc + t_cruise, t_total]
        vels  = [v_blur, v_max, v_max, v_blur]
        shape = "Trapezoid"

    else: #if a triangle
        v_peak = math.sqrt(v_blur**2 + acc * dist)
        t_acc = (v_peak - v_blur) / acc
        t_total = 2 * (v_peak - v_blur) / acc
        
        times = [0.0, t_acc, t_total]
        vels  = [v_blur, v_peak, v_blur]
        shape = "Triangle"
    
    return t_total, times, vels, shape
