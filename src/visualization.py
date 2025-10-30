"""Utility to visualize photo plans.
"""

import typing as T

import plotly.graph_objects as go
import plotly.express as px
import pandas as pd
from src.data_model import DatasetSpec, Waypoint
from src.plan_computation import apply_trapezoidal_prof

def plot_profiling(dist, acc, v_blur, v_max):
    """Graphing the velocity profile for trapezoidal/triangular (time on x-axis, velocity on y-axis).
    Args:
        dist: distance from one image point to another
        acc: acceleration of drone
        v_blur: calculated drone velocity for acceptable blur
        v_max: maximum drone speed
    Returns:
        Plotly figure object.
    """

    t_total, times, vels, shape = apply_trapezoidal_prof(dist, acc, v_blur, v_max)

    df = pd.DataFrame({"time_s": times, "speed_mps": vels})

    peak_txt = f" | Peak: {vels[1]:.3f} m/s" if shape == "Triangle" else ""
    fig = px.line(df, x = "time_s", y = "speed_mps", markers = True, title = f"{shape} profile", subtitle = f"Covered {round(dist, 3)}m in {round(t_total, 3)}s | Target Max: {round(v_max, 3)} m/s{peak_txt} |  Blur Velocity: {round(vels[0], 3)} m/s")

    fig.update_layout(xaxis_title="Time (s)", yaxis_title="Speed (m/s)")

    return fig

def plot_photo_plan(photo_plans: T.List[Waypoint], dataset_spec: DatasetSpec) -> go.Figure:
    """Plot the photo plan on a 2D grid.

    Args:
        photo_plans: List of waypoints for the photo plan.

    Returns:
        Plotly figure object.
    """
    
    #Create dataframe for waypoints
    rows = []
    for index, waypoint in enumerate(photo_plans):
        rows.append({"idx": index, "x-axis": waypoint.x, "y-axis": waypoint.y, "speed_mps": waypoint.speed_mps, "capture": waypoint.photo_trigger})

    df = pd.DataFrame(rows)
    fig = px.line(df, x = "x-axis", y = "y-axis", markers = True, color = "capture", title = "Drone Flight Plan", subtitle = f"Drone constant height at {waypoint.z}m") #Capture keeps the path color consistent, hover_data allows you to control aspects of mouse hover data

    #Creating bounding rectangle to show what region the photos need to be taken for
    fig.add_shape(type="rect", x0 = 0, y0 = 0, x1 = dataset_spec.scan_dimension_x, y1 = dataset_spec.scan_dimension_y, line = dict(dash = "dash")) #Adding a rectangle to show region of photo capture

    #Labeling start and end points
    start_x = df["x-axis"].iloc[0]
    start_y = df["y-axis"].iloc[0]
    end_x = df["x-axis"].iloc[-1]
    end_y = df["y-axis"].iloc[-1]

    fig.add_scatter(x = [start_x], y = [start_y], text = ["Starting Position"], textposition = "bottom right", name="Start")
    fig.add_scatter(x = [end_x],y = [end_y], text = ["Ending Position"], textposition = "bottom right", name="End")

    return fig