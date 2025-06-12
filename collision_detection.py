import math
from shapely.geometry import Polygon
import numpy as np # Added NumPy import

# A small epsilon value for float comparisons
EPSILON = 1e-9

def get_axes(polygon: Polygon) -> list[tuple[float, float]]:
    """
    Calculates the unique normalized normal vectors to the edges of a polygon.
    """
    axes_set = set() # Use a set to store unique axes

    if polygon is None or polygon.exterior is None or not polygon.exterior.coords:
        return []

    coords = list(polygon.exterior.coords)

    if len(coords) < 4: # Needs at least 3 distinct points for a triangle (P0,P1,P2,P0)
        return []

    for i in range(len(coords) - 1): # Iterate through edges
        p1 = coords[i]
        p2 = coords[i+1]

        edge_x = p2[0] - p1[0]
        edge_y = p2[1] - p1[1]

        axis_x = -edge_y # Normal vector (orthogonal)
        axis_y = edge_x

        length = math.sqrt(axis_x**2 + axis_y**2)
        if length < EPSILON: # Edge has zero length, skip
            continue

        axes_set.add((axis_x / length, axis_y / length)) # Add normalized axis

    return list(axes_set)

def project_polygon(axis: tuple[float, float], polygon_coords_np: np.ndarray) -> tuple[float, float] | None:
    """
    Projects a polygon (represented by its NumPy coordinate array) onto a given axis
    and returns the min and max projection values.
    Returns None if the polygon_coords_np is invalid or has no coordinates.
    """
    # polygon_coords_np is expected to be a NumPy array of shape (N, 2)
    if polygon_coords_np is None or not isinstance(polygon_coords_np, np.ndarray) or \
       polygon_coords_np.ndim != 2 or polygon_coords_np.shape[1] != 2:
        # Invalid input format for polygon coordinates
        return None

    if polygon_coords_np.shape[0] == 0: # No coordinates
        return None

    axis_np = np.array(axis)

    # Calculate all vertex projections onto the axis using NumPy's dot product
    projections = np.dot(polygon_coords_np, axis_np)

    # Find the min and max of these projections
    min_proj = projections.min()
    max_proj = projections.max()

    return min_proj, max_proj

def calculate_polygon_ttc(poly1: Polygon, vel1: tuple[float, float],
                          poly2: Polygon, vel2: tuple[float, float]) -> float:
    """
    Calculates the Time To Collision (TTC) for two convex polygons
    translating with given velocities. Uses the Separating Axis Theorem (SAT).
    Returns float('inf') if no collision is predicted.
    Returns 0.0 if polygons are already overlapping or touching and will collide.
    """
    if poly1 is None or poly2 is None:
        return float('inf') # Cannot operate on None polygons

    # Step 1: Initial Overlap Check for interior intersection
    if poly1.overlaps(poly2):
        return 0.0

    # Calculate relative velocity (poly2 relative to poly1)
    v_rel_x = vel2[0] - vel1[0]
    v_rel_y = vel2[1] - vel1[1]

    # Step 2: Handle zero relative velocity
    if abs(v_rel_x) < EPSILON and abs(v_rel_y) < EPSILON:
        return 0.0 if poly1.intersects(poly2) else float('inf')

    # --- START AABB BROAD PHASE ---
    # (This entire block replaces the previous faulty AABB filter)

    b1 = poly1.bounds # Expected format: (minx, miny, maxx, maxy)
    b2 = poly2.bounds

    if not b1 or len(b1) != 4 or not b2 or len(b2) != 4:
        # Invalid bounds data, implies degenerate polygons or issue with .bounds.
        return float('inf')

    # Nested helper function for 1D Time-To-Collision
    def solve_1d_ttc_detailed(p1_min_axis, p1_max_axis, p2_min_axis, p2_max_axis, rel_vel_on_axis):
        # EPSILON is assumed to be accessible from the outer scope
        if abs(rel_vel_on_axis) < EPSILON: # Stationary relative to each other on this axis
            if p1_max_axis < p2_min_axis - EPSILON or \
               p2_max_axis < p1_min_axis - EPSILON: # Separated and stationary
                return (float('inf'), float('-inf')) # No collision possible
            else: # Overlapping/touching and stationary
                return (float('-inf'), float('inf')) # Collision interval is indefinite
        else:
            t_reach_p1_min = (p1_min_axis - p2_max_axis) / rel_vel_on_axis
            t_reach_p1_max = (p1_max_axis - p2_min_axis) / rel_vel_on_axis

            if t_reach_p1_min < t_reach_p1_max:
                return (t_reach_p1_min, t_reach_p1_max)
            else:
                return (t_reach_p1_max, t_reach_p1_min)

    # Calculate TTC for X-axis
    tx_enter, tx_leave = solve_1d_ttc_detailed(b1[0], b1[2], b2[0], b2[2], v_rel_x)

    # Calculate TTC for Y-axis
    ty_enter, ty_leave = solve_1d_ttc_detailed(b1[1], b1[3], b2[1], b2[3], v_rel_y)

    # Combine 1D collision intervals
    aabb_ttc_overall_enter = max(tx_enter, ty_enter)
    aabb_ttc_overall_leave = min(tx_leave, ty_leave)

    # Check for non-collision based on combined AABB interval
    if aabb_ttc_overall_enter > aabb_ttc_overall_leave + EPSILON or \
       aabb_ttc_overall_leave < -EPSILON:
        return float('inf')

    # --- END AABB BROAD PHASE ---
    # If execution reaches here, AABBs might collide. Proceed to SAT.

    # Convert polygon coordinates to NumPy arrays for efficient projection
    # Convert Shapely coordinates to a list of Python tuples/lists
    p1_coords_list = list(poly1.exterior.coords)
    # Create a NumPy array using only the first two components (X, Y) of each point.
    # Include a check for points having at least 2 coordinates.
    coords1_np = np.array([(pt[0], pt[1]) for pt in p1_coords_list if len(pt) >= 2], dtype=float)

    # Convert Shapely coordinates to a list of Python tuples/lists
    p2_coords_list = list(poly2.exterior.coords)
    # Create a NumPy array using only the first two components (X, Y) of each point.
    coords2_np = np.array([(pt[0], pt[1]) for pt in p2_coords_list if len(pt) >= 2], dtype=float)

    # Check if coordinate lists became empty after filtering for 2D points
    if coords1_np.shape[0] == 0 or coords2_np.shape[0] == 0:
        # This implies original coordinates might have been malformed (e.g. all points < 2D)
        return float('inf')

    # Ensure NumPy arrays are valid for projection
    if not (coords1_np.ndim == 2 and coords1_np.shape[1] == 2 and coords1_np.shape[0] > 0 and \
            coords2_np.ndim == 2 and coords2_np.shape[1] == 2 and coords2_np.shape[0] > 0):
        return float('inf') # Invalid coordinate arrays

    axes1 = get_axes(poly1)
    axes2 = get_axes(poly2)

    # This temp_axes_map logic for unique axis orientations was present.
    # It is immediately followed by a simpler version if combined_axes_set is used.
    # For this optimization, we focus on using NumPy for projections,
    # assuming the axis generation logic (whichever is active) is separate.
    # The current file has both blocks, with the second one (combined_axes_set)
    # being the one that effectively defines `axes`.

    _temp_axes_map = {} # Renamed to avoid conflict if it was meant to be unused
    for ax_list in [axes1, axes2]:
        for ax in ax_list:
            rounded_ax_key = (round(ax[0], 8), round(ax[1], 8))
            if rounded_ax_key[1] < -EPSILON or (abs(rounded_ax_key[1]) < EPSILON and rounded_ax_key[0] < -EPSILON):
                opposite_rounded_ax_key = (-rounded_ax_key[0], -rounded_ax_key[1])
                if opposite_rounded_ax_key in _temp_axes_map:
                    continue
                else:
                    _temp_axes_map[rounded_ax_key] = ax
            elif rounded_ax_key not in _temp_axes_map:
                 _temp_axes_map[rounded_ax_key] = ax

    # This is the effective axis generation line from the previous version of the file
    combined_axes_set = set(axes1 + axes2)
    axes = list(combined_axes_set)


    if not axes:
        return float('inf')

    ttc_interval_start = 0.0
    ttc_interval_end = float('inf')

    for axis_tuple in axes: # Renamed axis to axis_tuple to avoid conflict with np.array(axis)
        # Pass NumPy coordinate arrays to project_polygon
        proj1_res = project_polygon(axis_tuple, coords1_np)
        proj2_res = project_polygon(axis_tuple, coords2_np)

        if proj1_res is None or proj2_res is None:
            return float('inf')

        p1_min, p1_max = proj1_res
        p2_min, p2_max = proj2_res

        # Use axis_tuple for dot product with velocities
        relative_velocity_on_axis = v_rel_x * axis_tuple[0] + v_rel_y * axis_tuple[1]

        if abs(relative_velocity_on_axis) < EPSILON:
            if p1_max < p2_min - EPSILON or p2_max < p1_min - EPSILON:
                return float('inf')
        else:
            t1 = (p1_min - p2_max) / relative_velocity_on_axis
            t2 = (p1_max - p2_min) / relative_velocity_on_axis

            current_axis_t_enter = min(t1, t2)
            current_axis_t_leave = max(t1, t2)

            ttc_interval_start = max(ttc_interval_start, current_axis_t_enter)
            ttc_interval_end = min(ttc_interval_end, current_axis_t_leave)

            if ttc_interval_start > ttc_interval_end + EPSILON:
                return float('inf')

    if ttc_interval_start < ttc_interval_end + EPSILON and ttc_interval_end >= -EPSILON:
        final_ttc = max(0.0, ttc_interval_start)

        if final_ttc < EPSILON:
            return 0.0 if poly1.intersects(poly2) else final_ttc

        return final_ttc
    else:
        return float('inf')
