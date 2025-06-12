# Algorithm: Calculating Time to Collision (TTC) between Two Polygons

**Objective:** Determine the time until two moving polygons (e.g., a Self-Driving Car and an Other Vehicle) will collide, if at all.

**Inputs:**
*   `poly1`: Polygon of the SDC (e.g., a rectangle representing its shape).
*   `vel1`: Velocity vector of the SDC (e.g., `(vx_sdc, vy_sdc)`).
*   `poly2`: Polygon of the OV.
*   `vel2`: Velocity vector of the OV (e.g., `(vx_ov, vy_ov)`).

---

## TTC Algorithm Steps:

1.  **Initial Overlap Check:**
    *   **Action:** Check if `poly1` (SDC) and `poly2` (OV) are already overlapping.
    *   **Result:** If overlapping, TTC is 0.0. Algorithm ends.
    *   **Image:** See `images/step_1_image_description.txt` (Description of visual: SDC and OV polygons shown partially intersecting).

2.  **Calculate Relative Velocity:**
    *   **Action:** Determine the velocity of the OV relative to the SDC. This simplifies the problem to a stationary SDC and a moving OV. `v_rel = vel2 - vel1`.
    *   **Image:** See `images/step_2_image_description.txt` (Description of visual: SDC stationary, arrow showing relative velocity from OV).

3.  **Zero Relative Velocity Check:**
    *   **Action:** If the relative velocity is (near) zero, the polygons are moving together or are both stationary relative to each other.
    *   **Result:** If they are not touching/overlapping, TTC is infinite (no collision). If they *are* touching, TTC is 0.0. Algorithm may end.
    *   **Image:** See `images/step_3_image_description.txt` (Description of visual: SDC and OV separated, no relative velocity arrow).

4.  **Broad Phase: Axis-Aligned Bounding Boxes (AABBs) - Setup:**
    *   **Action:** For a quick preliminary check, find the smallest upright rectangles (AABBs) that enclose the SDC (`aabb1`) and the OV (`aabb2`).
    *   **Image:** See `images/step_4_image_description.txt` (Description of visual: SDC and OV polygons with their AABBs).

5.  **Broad Phase: AABB Collision on X-axis:**
    *   **Action:** Considering only the X-coordinates of the AABBs and their relative X-velocity, calculate the time interval (`tx_enter`, `tx_leave`) during which the AABBs would overlap on the X-axis.
    *   **Image:** See `images/step_5_image_description.txt` (Description of visual: 1D X-axis projections of AABBs and relative movement).

6.  **Broad Phase: AABB Collision on Y-axis:**
    *   **Action:** Similarly, considering only the Y-coordinates of the AABBs and their relative Y-velocity, calculate the time interval (`ty_enter`, `ty_leave`) during which the AABBs would overlap on the Y-axis.
    *   **Image:** See `images/step_6_image_description.txt` (Description of visual: 1D Y-axis projections of AABBs and relative movement).

7.  **Broad Phase: Combine AABB Collision Intervals:**
    *   **Action:** Find the overall time interval when the AABBs would be colliding: `aabb_ttc_enter = max(tx_enter, ty_enter)`, `aabb_ttc_leave = min(tx_leave, ty_leave)`.
    *   **Result:** If `aabb_ttc_enter > aabb_ttc_leave` or `aabb_ttc_leave < 0`, the AABBs will not collide. TTC is infinite. Algorithm ends.
    *   **Image:** See `images/step_7_image_description.txt` (Description of visual: Timeline graphic showing intersection of X and Y AABB collision intervals).

8.  **Narrow Phase: Get Separation Axes (SAT):**
    *   **Action:** If AABBs *might* collide, proceed to the Separating Axis Theorem (SAT). Find all unique normal vectors to the edges of both the SDC and OV polygons.
    *   **Image:** See `images/step_8_image_description.txt` (Description of visual: SDC and OV polygons with normal vectors on edges).

9.  **Narrow Phase: Projection and Interval Update (Loop Setup):**
    *   **Action:** Initialize overall `ttc_interval_start = 0.0` and `ttc_interval_end = infinity`. Iterate through each separation axis.
    *   **Image:** See `images/step_9_image_description.txt` (Description of visual: Diagram indicating loop start with axes list and initial TTC interval values).

10. **Narrow Phase: Per-Axis Collision Check (Inside Loop):**
    *   **Action (For each axis):**
        a.  Project both SDC and OV polygons onto the current axis.
        b.  Project the relative velocity (`v_rel`) onto this axis.
        c.  If projected relative velocity is near zero: If projected 1D segments are separated, TTC is infinite. Algorithm ends.
        d.  Else: Calculate time interval (`current_axis_t_enter`, `current_axis_t_leave`) for overlap of 1D segments.
        e.  Update overall collision window: `ttc_interval_start = max(ttc_interval_start, current_axis_t_enter)`, `ttc_interval_end = min(ttc_interval_end, current_axis_t_leave)`.
        f.  If `ttc_interval_start > ttc_interval_end`, a separating axis is found. TTC is infinite. Algorithm ends.
    *   **Image:** See `images/step_10_image_description.txt` (Description of visual: Example for one axis - projection, relative velocity on axis, interval update).

11. **Narrow Phase: Final TTC Decision:**
    *   **Action:** After checking all axes: If `ttc_interval_start <= ttc_interval_end` AND `ttc_interval_end >= 0`, collision is possible.
    *   **Image:** See `images/step_11_image_description.txt` (Description of visual: Final timeline showing `ttc_interval_start` and `ttc_interval_end`).

12. **Determine Final TTC Value:**
    *   **Action:** `final_ttc = max(0.0, ttc_interval_start)`. If `final_ttc` is near 0, re-verify with an intersection test.
    *   **Image:** See `images/step_12_image_description.txt` (Description of visual: SDC and OV at moment of collision, or indication of no future collision).
