# Separating Axis Theorem (SAT)

The Separating Axis Theorem (SAT) is a fundamental concept in collision detection, particularly for convex polygons. It states that two convex objects do not overlap if there exists a line (or axis) onto which the projections of the objects do not overlap.

## Collision Detection with Convex Polygons

SAT is an efficient algorithm for determining whether two convex polygons are colliding. The core idea is to find a "separating axis" – an axis where the projections of the two polygons do not overlap. If such an axis is found, the polygons are not colliding. If no such axis exists after checking all potential separating axes, then the polygons are colliding.

## Projecting Polygons onto Axes

To apply SAT, we need to consider a set of potential separating axes. For convex polygons, these axes are the lines perpendicular to each edge of both polygons.

For each potential separating axis:
1. **Project** both polygons onto the axis. This means finding the minimum and maximum extent of each polygon along that axis. This creates two 1D intervals on the axis.
2. **Check for overlap** between these two intervals. If the intervals do not overlap, then the axis is a separating axis, and the polygons are not colliding. We can stop and conclude there is no collision.

If all potential separating axes have been checked and none of them are separating axes (i.e., the projections overlap on every axis), then the polygons are colliding.

# AABB Broad Phase Collision Detection

## Axis-Aligned Bounding Boxes (AABB)

An Axis-Aligned Bounding Box (AABB) is a rectangular box that encloses an object, with its sides aligned with the coordinate axes (e.g., x-axis and y-axis in 2D). It is defined by the minimum and maximum coordinates of the object along each axis. AABBs are simple to create and very fast to check for overlaps.

## Broad Phase Optimization

Collision detection can be computationally expensive, especially when dealing with many objects or complex shapes. The AABB broad phase is a common optimization technique used to quickly eliminate pairs of objects that are clearly not colliding, before performing more precise (and expensive) collision checks like the Separating Axis Theorem (SAT).

The process is as follows:
1. **AABB Test:** For each pair of objects, first perform a quick overlap test on their AABBs.
    - Two AABBs overlap if and only if their projections onto all coordinate axes overlap. For 2D, this means the x-intervals overlap AND the y-intervals overlap.
    - If the AABBs do not overlap, the enclosed objects cannot be colliding, so we can skip any further checks for this pair.
2. **Narrow Phase (e.g., SAT):** If the AABBs *do* overlap, it means the objects *might* be colliding. Only then do we proceed to a more precise collision detection algorithm, like SAT, to determine if they actually are.

Using AABB as a broad phase significantly reduces the number of times the more complex SAT algorithm needs to be run, improving overall performance, especially in scenarios with many objects.

# Time To Collision (TTC)

Time To Collision (TTC), also known as Time of Impact (TOI), is a measure used in physics simulations and robotics to determine how long it will take for two moving objects to collide, assuming their velocities remain constant.

## Relative Velocities

To calculate TTC, we typically work with the relative velocity between the two objects. If object A has velocity `vA` and object B has velocity `vB`, their relative velocity `vRel` is `vB - vA` (or `vA - vB`, depending on the reference frame, but consistency is key). This simplifies the problem by considering one object as stationary and the other moving with `vRel`.

## TTC Algorithm with SAT Projections

For convex polygons, TTC can be calculated by extending the Separating Axis Theorem (SAT). The core idea is to find the first time of contact along any potential separating axis.

1.  **Identify Potential Separating Axes:** Just like in SAT, these are the axes perpendicular to the edges of both polygons.
2.  **For each axis:**
    *   Project the two polygons (P1 and P2) onto the axis. Let their initial projections be intervals `[min1, max1]` and `[min2, max2]`.
    *   Project the relative velocity `vRel` onto the axis. Let this projected relative speed be `vRel_proj`.
    *   **Calculate the distance between the interval ends.** There are two potential collision scenarios on this axis:
        *   P1 approaching P2 from the "left": `distance1 = min2 - max1` (gap closes if `max1` is moving towards `min2`).
        *   P2 approaching P1 from the "left": `distance2 = min1 - max2` (gap closes if `max2` is moving towards `min1`).
    *   **Calculate TTC for this axis:**
        *   If `vRel_proj > 0` (P1 is moving towards P2 on this axis for `distance1` scenario):
            *   `ttc_axis1 = distance1 / vRel_proj`. If `distance1 < 0` (already overlapping), this could be negative.
        *   If `vRel_proj < 0` (P2 is moving towards P1 on this axis for `distance2` scenario, or P1 moving away from P2 for `distance1`):
            *   `ttc_axis2 = distance2 / (-vRel_proj)`. If `distance2 < 0`, this could be negative.
        *   We are interested in positive TTC values that represent future collisions.
        *   If `vRel_proj` is close to zero, they are not approaching each other on this axis, so TTC is effectively infinite for this closing direction.
    *   Keep track of the *entry time* (`t_entry`) and *exit time* (`t_exit`) of overlap on this axis.
        *   `t_entry` is when the projections start to overlap.
        *   `t_exit` is when the projections cease to overlap.
        *   If `distance1 >= 0` (initially separated):
            *   `t_entry_axis = distance1 / vRel_proj` (if `vRel_proj > 0`)
            *   `t_exit_axis = (max2 - min1) / vRel_proj` (if `vRel_proj > 0`)
        *   If `distance2 >= 0` (initially separated):
            *   `t_entry_axis = distance2 / (-vRel_proj)` (if `vRel_proj < 0`)
            *   `t_exit_axis = (max1 - min2) / (-vRel_proj)` (if `vRel_proj < 0`)
        *   If they are already overlapping (`distance1 < 0` or `distance2 < 0`), `t_entry_axis` is 0.
3.  **Overall TTC:**
    *   We need to find the *latest* `t_entry` across all axes and the *earliest* `t_exit` across all axes.
    *   Let `max_t_entry` be the maximum of all `t_entry_axis` values calculated.
    *   Let `min_t_exit` be the minimum of all `t_exit_axis` values calculated.
    *   If `max_t_entry < min_t_exit` and `max_t_entry > 0`, then a collision will occur, and the TTC is `max_t_entry`.
    *   If `max_t_entry >= min_t_exit` or `max_t_entry < 0` (or no approach on any axis), then they are either already overlapping, moving away from each other, or will not collide. If they are already overlapping (initial SAT check fails), TTC is 0. If they are moving away, TTC can be considered infinite.

This algorithm determines the *first time of contact*. If the objects are already overlapping, the TTC is 0. If they are moving away from each other such that their projections will never overlap, or are parallel and will never meet, the TTC can be considered infinite. The smallest positive TTC calculated across all relevant axes is the actual Time To Collision for the objects.

# Polygon Examples for Collision Detection

This section provides examples of how to define convex polygons that can be used with the collision detection algorithms described (SAT, AABB, TTC). We will use `shapely.geometry.Polygon` for defining these polygons in Python.

```python
from shapely.geometry import Polygon

# Example: Define a polygon
# polygon = Polygon([(x1, y1), (x2, y2), ..., (xn, yn)])
```

## Vehicle-like Polygon Shapes

Here are a few examples of simple, convex, vehicle-like polygons. For SAT to work correctly, polygons must be convex.

### 1. Simple Rectangular Vehicle (e.g., a basic truck or bus)

This is the simplest representation, essentially a box.

**Vertices:** `[(0, 0), (0, 2), (5, 2), (5, 0)]`

**Python (Shapely):**
```python
vehicle_rect = Polygon([(0, 0), (0, 2), (5, 2), (5, 0)])
```

**ASCII Art:**
```
+-------+  (width 5, height 2)
|       |
+-------+
```
*(Note: ASCII art is a rough approximation and might not perfectly scale to vertex coordinates.)*

### 2. Basic Car Shape (Trapezoid-like front)

A slightly more detailed shape, still convex.

**Vertices:** `[(0, 0), (0, 1.5), (1, 2), (4, 2), (5, 1.5), (5, 0)]`

**Python (Shapely):**
```python
vehicle_car_basic = Polygon([
    (0, 0), (0, 1.5), (1, 2), (4, 2), (5, 1.5), (5, 0)
])
```

**ASCII Art:**
```
  .-----.   (Top edge from (1,2) to (4,2))
 /       \
+---------+   (Bottom edge from (0,0) to (5,0))
```

### 3. Compact Car Shape (More rounded front)

This shape uses more vertices to give a slightly more "aerodynamic" or compact car feel, while remaining convex.

**Vertices:** `[(0, 0), (0, 1), (0.5, 1.5), (1.5, 1.8), (3.5, 1.8), (4.5, 1.5), (5, 1), (5, 0)]`

**Python (Shapely):**
```python
vehicle_compact = Polygon([
    (0, 0), (0, 1), (0.5, 1.5), (1.5, 1.8),
    (3.5, 1.8), (4.5, 1.5), (5, 1), (5, 0)
])
```

**ASCII Art (Conceptual):**
```
   .--~~--.
  /        \
 +----------+
```
*(This ASCII art is highly conceptual as precise representation of many vertices is hard.)*

### 4. Simple Bicycle/Motorcycle (Narrow Rectangle)

Representing very narrow vehicles.

**Vertices:** `[(0, 0), (0, 0.5), (3, 0.5), (3, 0)]`

**Python (Shapely):**
```python
vehicle_bike = Polygon([(0, 0), (0, 0.5), (3, 0.5), (3, 0)])
```

**ASCII Art:**
```
+-----+ (width 3, height 0.5)
+-----+
```

These examples can be used as a starting point for creating vehicle representations in a simulation. Remember that the vertices must be listed in a consistent order (clockwise or counter-clockwise) and the polygon must be convex for SAT to function as expected.
The `shapely` library provides many useful functions for working with these geometric objects, such as calculating areas, centroids, intersections, etc., which can be helpful in a broader simulation context beyond just collision detection.

# Key Function Explanations

This section details the core functions used in the SAT and TTC calculation logic. It assumes that polygon coordinates are extracted from Shapely polygons and converted to NumPy arrays where necessary (e.g., `np.array(polygon.exterior.coords)`).

## 1. `get_axes(polygon_coords_np)`

*   **Purpose:** Calculates the potential separating axes for a given polygon. For convex polygons, these axes are the lines perpendicular to each edge of the polygon.
*   **Inputs:**
    *   `polygon_coords_np` (numpy.ndarray): A NumPy array of shape `(N, 2)` representing the ordered vertices (x, y coordinates) of the polygon. `N` is the number of vertices.
*   **Processing:**
    1.  Iterates through each edge of the polygon (from vertex `i` to vertex `i+1`).
    2.  For each edge, calculates the edge vector.
    3.  Finds the normal vector to this edge (perpendicular vector). This is typically done by swapping the components of the edge vector and negating one of them (e.g., if edge vector is `(dx, dy)`, normal is `(-dy, dx)` or `(dy, -dx)`).
    4.  Normalizes the normal vector (divides by its magnitude) to get a unit vector. This ensures consistency in projections.
*   **Outputs:**
    *   `list` of `numpy.ndarray`: A list where each element is a 1D NumPy array of shape `(2,)` representing a unique, normalized separating axis `[axis_x, axis_y]`. Duplicate axes (e.g., parallel axes from opposite edges) are usually avoided.

## 2. `project_polygon(axis, polygon_coords_np)`

*   **Purpose:** Projects a polygon onto a given axis. This results in a 1D interval (minimum and maximum scalar values) representing the "shadow" of the polygon on that axis.
*   **Inputs:**
    *   `axis` (numpy.ndarray): A 1D NumPy array of shape `(2,)` representing the normalized axis `[axis_x, axis_y]` onto which the polygon will be projected.
    *   `polygon_coords_np` (numpy.ndarray): A NumPy array of shape `(N, 2)` representing the ordered vertices (x, y coordinates) of the polygon.
*   **Processing:**
    1.  Calculates the dot product of each vertex of the polygon with the given axis. The dot product `v · a` projects vector `v` onto axis `a`.
    2.  Finds the minimum and maximum values from these dot products.
*   **Outputs:**
    *   `tuple` of `float`: A tuple `(min_projection, max_projection)` representing the minimum and maximum scalar values of the polygon's projection onto the axis.

## 3. `calculate_polygon_ttc(poly1_coords, vel1_coords, poly2_coords, vel2_coords)`

*   **Purpose:** Calculates the Time To Collision (TTC) between two moving convex polygons. It determines if and when they will collide if they continue moving at their current velocities.
*   **Inputs:**
    *   `poly1_coords` (numpy.ndarray): Vertices of the first polygon, shape `(N, 2)`.
    *   `vel1_coords` (numpy.ndarray): Velocity vector `[vx, vy]` of the first polygon, shape `(2,)`.
    *   `poly2_coords` (numpy.ndarray): Vertices of the second polygon, shape `(M, 2)`.
    *   `vel2_coords` (numpy.ndarray): Velocity vector `[vx, vy]` of the second polygon, shape `(2,)`.
*   **Processing:**
    1.  **Calculate Relative Velocity:** `relative_velocity = vel1_coords - vel2_coords`. (Note: the implementation might use `vel2 - vel1`; the key is consistency with how distances are interpreted).
    2.  **Get All Potential Separating Axes:** Collect all unique axes from `get_axes(poly1_coords)` and `get_axes(poly2_coords)`.
    3.  **Initialize TTC variables:** `ttc_max_entry_time = -infinity`, `ttc_min_exit_time = +infinity`.
    4.  **Iterate Through Each Axis:**
        *   Project both polygons onto the current axis using `project_polygon`, getting `(min1, max1)` and `(min2, max2)`.
        *   Project the `relative_velocity` onto the axis: `v_rel_proj = numpy.dot(relative_velocity, axis)`.
        *   **Collision Check (Static Overlap):** If `max1 < min2` or `max2 < min1` (the projections don't overlap) AND `v_rel_proj` is such that they are not moving towards each other on this axis (e.g., if `max1 < min2` and `v_rel_proj <= 0`), then this is a separating axis, and they will not collide (at least not initiated along this axis). If this holds for any axis, return `float('inf')` (or a large number) for no collision, unless an earlier check showed they are already overlapping (TTC=0).
        *   **Calculate Time of Entry and Exit for this Axis:**
            *   If `v_rel_proj == 0` (parallel movement or stationary relative to axis): If they are not already overlapping on this axis, they will never collide on it (`t_entry = inf`, `t_exit = inf`). If they *are* overlapping, they will continue to do so (`t_entry = -inf`, `t_exit = inf`).
            *   Otherwise, calculate `t_entry` (time when projections start overlapping) and `t_exit` (time when projections stop overlapping).
                *   `gap_before_entry1 = min2 - max1`
                *   `gap_before_entry2 = min1 - max2` (one of these will be the actual gap, the other will be negative if they overlap)
                *   If `v_rel_proj > 0` (poly1 moving towards poly2 on this axis):
                    *   `t_entry = gap_before_entry1 / v_rel_proj`
                    *   `t_exit = (max2 - min1) / v_rel_proj`
                *   Else (`v_rel_proj < 0`):
                    *   `t_entry = gap_before_entry2 / v_rel_proj` (note: `v_rel_proj` is negative, so `t_entry` can be positive)
                    *   `t_exit = (max1 - min2) / v_rel_proj`
            *   Ensure `t_entry` and `t_exit` are ordered correctly (swap if `t_entry > t_exit`).
        *   Update overall `ttc_max_entry_time = max(ttc_max_entry_time, t_entry)`
        *   Update overall `ttc_min_exit_time = min(ttc_min_exit_time, t_exit)`
    5.  **Determine Final TTC:**
        *   If `ttc_max_entry_time < ttc_min_exit_time` AND `ttc_max_entry_time >= 0`:
            *   A collision will occur. The TTC is `ttc_max_entry_time`.
            *   If `ttc_max_entry_time < 0` (which can happen if they were already overlapping and are moving further into overlap), this means they are already overlapping. If the initial projections overlapped, TTC should be 0. A robust implementation checks for initial overlap first. If `ttc_max_entry_time` is negative but they weren't initially overlapping, this implies they are moving apart from a point of future collision.
        *   Else (if `ttc_max_entry_time >= ttc_min_exit_time` or `ttc_max_entry_time < 0` and not initially overlapping): No future collision will occur. Return `float('inf')`.
    6.  **Initial Overlap Check:** Before iterating, a simple SAT check (projections overlapping on all axes with `v_rel_proj = 0`) should be done. If they are already overlapping, TTC is 0.
*   **Outputs:**
    *   `float`: The Time To Collision. Returns `0.0` if already overlapping, `float('inf')` if no collision is predicted, or the positive time in seconds (or other consistent unit) until collision.

*(Note: The exact implementation details for `calculate_polygon_ttc` can be complex, especially handling all edge cases like parallel movement, initial overlap, and objects moving away. The description above outlines the general approach based on finding the interval of collision times over all axes.)*

# Usage Example

Here's a basic example of how to use the `calculate_polygon_ttc` function with `shapely` polygons.
Assume `calculate_polygon_ttc`, `get_axes`, and `project_polygon` are defined as described, and `numpy` is imported as `np`.

```python
import numpy as np
from shapely.geometry import Polygon

# Assume the following functions are defined elsewhere in your project:
# - get_axes(polygon_coords_np) -> list_of_axes
# - project_polygon(axis, polygon_coords_np) -> (min_proj, max_proj)
# - calculate_polygon_ttc(poly1_coords_np, vel1_np, poly2_coords_np, vel2_np) -> float (ttc)
# For this example, we'll use placeholder stubs if these functions are not yet implemented.

# Placeholder for actual implementation if not available:
def get_axes(polygon_coords_np):
    # Simplified: In a real scenario, this calculates normals to edges
    # For example purposes, let's return fixed axes for a square
    # This is NOT a correct general implementation.
    return [np.array([1, 0]), np.array([0, 1])]

def project_polygon(axis, polygon_coords_np):
    # Simplified: In a real scenario, this projects all vertices
    # This is NOT a correct general implementation.
    projections = [np.dot(vertex, axis) for vertex in polygon_coords_np]
    return min(projections), max(projections)

def calculate_polygon_ttc(poly1_coords, vel1, poly2_coords, vel2):
    """
    Placeholder for the actual TTC calculation logic.
    This function would use get_axes and project_polygon internally.
    For the sake of this example, let's imagine it's implemented
    and can return a value. A real implementation is complex.
    If you have the actual 'collision_ttc.py' or similar, import from it.
    """
    # This is a MOCK implementation for demonstration only.
    # Replace with actual call to your implemented TTC function.
    print("Note: Using MOCK calculate_polygon_ttc for example.")

    # Basic initial overlap check (very simplified)
    axes1 = get_axes(poly1_coords)
    axes2 = get_axes(poly2_coords)
    all_axes = axes1 + axes2 # In reality, remove duplicates

    for axis in all_axes:
        p1_min, p1_max = project_polygon(axis, poly1_coords)
        p2_min, p2_max = project_polygon(axis, poly2_coords)
        if p1_max < p2_min or p2_max < p1_min:
            # If separated on any axis, they are not currently colliding
            pass # Continue to TTC calculation based on velocity

    # Simplified TTC logic:
    # Consider only movement along x-axis for this mock example
    relative_velocity_x = vel1[0] - vel2[0]

    # Get x-bounds
    p1_x_coords = poly1_coords[:, 0]
    p2_x_coords = poly2_coords[:, 0]
    p1_min_x, p1_max_x = np.min(p1_x_coords), np.max(p1_x_coords)
    p2_min_x, p2_max_x = np.min(p2_x_coords), np.max(p2_x_coords)

    if relative_velocity_x > 0: # poly1 moving towards poly2 (rightwards)
        gap = p2_min_x - p1_max_x
        if gap < 0: return 0.0 # Already overlapping
        return gap / relative_velocity_x if gap >=0 else float('inf')
    elif relative_velocity_x < 0: # poly2 moving towards poly1 (poly1 leftwards relative to poly2)
        gap = p1_min_x - p2_max_x
        if gap < 0: return 0.0 # Already overlapping
        return gap / relative_velocity_x if gap >=0 else float('inf') # result will be positive
    else:
        # If no relative x-velocity, and they didn't overlap initially on x,
        # they won't collide along x. (This ignores y-axis completely)
        if p1_max_x < p2_min_x or p2_max_x < p1_min_x:
             return float('inf')
        else: # Overlapping on x, no relative x-movement. Check y.
             # For this mock, we assume they are colliding if x overlaps and no x_vel
             return 0.0


# 1. Create two Shapely Polygons
# Polygon 1: A square
poly1_shape = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
# Polygon 2: Another square, initially to the right of Polygon 1
poly2_shape = Polygon([(5, 0), (5, 1), (6, 1), (6, 0)])

# Convert Shapely polygons to NumPy arrays of coordinates
# Shapely's exterior.coords gives N+1 points (last point repeats first), so slice [:-1]
poly1_coords_np = np.array(poly1_shape.exterior.coords[:-1])
poly2_coords_np = np.array(poly2_shape.exterior.coords[:-1])

# 2. Define their velocities (as NumPy arrays [vx, vy])
# Polygon 1 is moving right, Polygon 2 is stationary
velocity1_np = np.array([1.0, 0.0])  # Moves 1 unit per second in x
velocity2_np = np.array([0.0, 0.0])  # Stationary

# 3. Call calculate_polygon_ttc
# Ensure your actual 'calculate_polygon_ttc' function is imported or defined
ttc = calculate_polygon_ttc(poly1_coords_np, velocity1_np, poly2_coords_np, velocity2_np)

# 4. Print or interpret the result
print(f"Polygon 1 vertices: {poly1_coords_np.tolist()}")
print(f"Polygon 1 velocity: {velocity1_np.tolist()}")
print(f"Polygon 2 vertices: {poly2_coords_np.tolist()}")
print(f"Polygon 2 velocity: {velocity2_np.tolist()}")

if ttc == 0.0:
    print("Result: Polygons are already overlapping.")
elif ttc == float('inf'):
    print("Result: Polygons will not collide with current velocities.")
else:
    print(f"Result: Time To Collision (TTC) is {ttc:.2f} seconds.")

# Example 2: Head-on collision
poly3_shape = Polygon([(0,0), (1,0), (1,1), (0,1)])
poly3_coords_np = np.array(poly3_shape.exterior.coords[:-1])
vel3_np = np.array([2.0, 0.0]) # Moving right

poly4_shape = Polygon([(10,0), (11,0), (11,1), (10,1)])
poly4_coords_np = np.array(poly4_shape.exterior.coords[:-1])
vel4_np = np.array([-1.0, 0.0]) # Moving left

ttc_2 = calculate_polygon_ttc(poly3_coords_np, vel3_np, poly4_coords_np, vel4_np)
print(f"\nExample 2 TTC: {ttc_2}") # Expected: (10 - 1) / (2 - (-1)) = 9 / 3 = 3 seconds


# Example 3: Already overlapping
poly5_shape = Polygon([(0,0), (2,0), (2,2), (0,2)])
poly5_coords_np = np.array(poly5_shape.exterior.coords[:-1])
vel5_np = np.array([1.0, 0.0])

poly6_shape = Polygon([(1,1), (3,1), (3,3), (1,3)]) # Overlaps (1,1) to (2,2)
poly6_coords_np = np.array(poly6_shape.exterior.coords[:-1])
vel6_np = np.array([0.0, 0.0])

ttc_3 = calculate_polygon_ttc(poly5_coords_np, vel5_np, poly6_coords_np, vel6_np)
print(f"Example 3 TTC (overlap): {ttc_3}") # Expected: 0.0


# Example 4: Moving away
poly7_shape = Polygon([(0,0), (1,0), (1,1), (0,1)])
poly7_coords_np = np.array(poly7_shape.exterior.coords[:-1])
vel7_np = np.array([-1.0, 0.0]) # Moving left

poly8_shape = Polygon([(5,0), (6,0), (6,1), (5,1)])
poly8_coords_np = np.array(poly8_shape.exterior.coords[:-1])
vel8_np = np.array([1.0, 0.0]) # Moving right

ttc_4 = calculate_polygon_ttc(poly7_coords_np, vel7_np, poly8_coords_np, vel8_np)
print(f"Example 4 TTC (moving away): {ttc_4}") # Expected: inf
```

This example provides a clear, runnable template (assuming the core TTC logic exists) for users to understand how to integrate the TTC calculation into their own projects. Remember to replace the placeholder/mock `calculate_polygon_ttc` with the actual, fully implemented version for correct results.
The `shapely` library is used for convenient polygon definition, and `numpy` is used for coordinate and vector representations, which is typical in such geometric calculations.
