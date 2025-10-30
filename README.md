## Drone Trajectory Planner
![FlightPlanner](/assets/droneflightplanex1.jpg)
Example of generated boustrophedon drone flight pattern
![TrapezoidalProfiling](/assets/droneflightplanex2.jpg)
Example of generated trapezoidal profiling for drone velocity
![TriangularProfiling](/assets/droneflightplanex3.jpg)
Example of generated triangular profiling for drone velocity
Plan, simulate, and visualize drone photo-capture flight over a rectangular area using boustrophedon pattern. Given a camera model and user dataset specs (overlap, sidelap, height, scan size, exposure), the project:
- computes image spacing from camera geometry
- lays out capture waypoints in a serpentine grid
- assigns a safe capture speed from GSD & exposure (motion-blur bound)
- computes segment timing with a trapezoidal/triangular velocity profile,
- visualizes both the 2D flight path and the velocity profile (using plotly express)

**Conventions & Units**
- World frame: flat ground plane, nadir-pointing camera, canonical pose
- Units: distances in meters, sensor sizes in mm, focal lengths in pixels, image sizes in pixels.
- Overlap & sidelap are ratios in [0, 1].

**Influencing factors:**
- Change exposure --> changes v_blur (and timing/velocity profiles), not waypoint positions
- Change height --> scales footprint & spacing --> changes waypoint density (sparse or populated)
- Change overlap/sidelap --> changes spacing (denser grid = higher overlap).
- Change accel/v_max --> changes profile shape (triangle vs trapezoid) and segment times.

**Dev Notes:**
- Python ≥ 3.9, numpy, pandas, plotly
- Jupyter users: set plotly renderer (pio.renderers.default = "vscode") if not generating properly
- Drone trajectory planner system for the [build project](https://hub.buildfellowship.com/projects/drone-flight-planner-system-flight-path-for-efficient-data-capture)