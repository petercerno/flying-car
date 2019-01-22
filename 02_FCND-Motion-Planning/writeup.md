## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

The starter code consists of two files: `motion_planning.py` and
`planning_utils.py`. I have verified that both scripts work out-of-the-box.

#### `motion_planning.py`

This file contains the main implementation of the drone (inside the class
`MotionPlanning`). The drone works as follows:

Upon initialization, the drone sets `in_mission` to `True` (which enables the
script to update the drone's internal state), the local `target_position`
`(north, east, altitude, heading)` to `(0, 0, 0, 0)`, and the drone's internal
state to the initial `MANUAL` state.

Then the script proceeds with the `arming_transition`, which sets the drone's
internal state to `ARMING`, arms the drone, and switches it to the guided
(autonomous) mode.

When the drone is armed, the script proceeds with the `PLANNING` state by
calling the method: `plan_path`. This method reads `colliders.csv` describing
the environment (2.5D map of obstacles) and then creates a simplified 2D `grid`
(2D configuration space, where the drone can safely fly) based on the given
obstacle data, drone altitude (set to a predefined constant: 5m above ground),
and safety distance around the obstacles. A* algorithm is called to find the
lowest cost path inside the `grid` (consisting only of the safe-to-fly cells)
based on some predefined start and goal cells, and then this path is converted
to a list of local waypoints.

Then the script proceeds with the `takeoff_transition`, which sets the drone's
internal state to `TAKEOFF` and takes off the drone to the predefined altitude
(stored in the current local `target_position`).

When the drone gets close enough to its `target_position` (i.e. having the
altitude at least 95% of the target altitude), it proceeds with the
`waypoint_transition`. This sets the drone to the `WAYPOINT` state and then
the script simply navigates the drone consecutively across all waypoints in the
given order (similarly as in the `backyard_flyer_solution.py`).

When the drone finally gets to the last waypoint of the plan and its velocity
is small enough, the `landing_transition` is executed. This transition simply
sets the drone's internal state to `LANDING` state and lands the drone.

Finally, when the drone is close enough to the home position and close enough
to the ground, `disarming_transition` is executed, which sets the drone's
internal state to `DISARMING` state, disarms the drone, and switches it to
the manual mode.

When the drone becomes disarmed and in the manual mode, the `manual_transition`
switches its internal state to the initial `MANUAL` state, disconnects the
script from the drone, and sets the `in_mission` to `False` to disable any
follow-up internal state transitions.

#### `MotionPlanning` VS `BackyardFlyer`

The main difference between the `MotionPlanning` drone and the `BackyardFlyer`
drone is that the `MotionPlanning` drone has one additional state: `PLANNING`,
which is responsible for creating a plan (list of waypoints) for the drone.
The `PLANNING` state is injected right after the `ARMING` state.
The `BackyardFlyer` drone does not have the `PLANNING` state, because it
pre-defines the list of waypoints during its initialization.

Note that the `MotionPlanning` drone could also create the plan during
its initialization phase (since the 2.5D environment is fixed). However, the
current design (with the `PLANNING` state) is slightly better, as it allows the
drone to be modified to do regular re-plannings (plan adjustments) for dynamic
environments.

#### `planning_utils.py`

This file contains some auxiliary methods used in the `motion_planning.py`:

* `create_grid`: Creates a simplified 2D `grid` (2D configuration space, where
the drone can safely fly) based on the given obstacle data, drone altitude, and
safety distance around the obstacles. This `grid` is essentially an
approximation of the intersection of obstacles (enlarged by the given safety 
distance) and a horizontal plane hovering above the ground with the given
altitude. Every cell in this `grid` is either `0` or `1` depending on whether
the cell is safe or not for the drone to fly. The safety distance is added
to the obstacles in order to account for the size of the drone.
* `valid_actions`: This method is used in the A* algorithm for finding valid
state transitions. For a given grid and node, it returns a list of valid
actions that can be applied to the given node.
* `a_star`: A concrete A* implementation that finds the lowest cost path inside
the given grid from the given start cell to the given end cell. This path
consists only of safe-to-fly cells (with value `0`).
* `heuristic`: Simple Euclidean heuristic used to speed-up the A* algorithm.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

I have simply read the first line of `'colliders.csv'` and split it into two
string parts (separated by comma).

```
with open('colliders.csv') as colliders_file:
    line = next(colliders_file)
    lat_lon_str = line.split(',')
```

Then I split both latitude and longitude substrings (by whitespace) and
extracted the floats representing the global position:

```
    lat0 = float(lat_lon_str[0].strip().split()[1])
    lon0 = float(lat_lon_str[1].strip().split()[1])
    self.set_home_position(lon0, lat0, 0)
```

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

The `local_position` can be computed as follows:

```
local_position = global_to_local(self.global_position, self.global_home)
```

However, the property `self.local_position` already contains the correct local
position (it was updated when we called `self.set_home_position`) so no need to
compute it explicitly.

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

Assuming that the global home position (obtained from the `'colliders.csv'`
file) is defined as the center of the map, i.e. the grid cell
`(-north_offset, -east_offset)`, we only need to add the local position offset
to this grid cell to get the grid start position.

```
grid_start = (
    -north_offset + int(self.local_position[0]),
    -east_offset + int(self.local_position[1]))
```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

If `lon_lat_goal` contains the geodetic coords of the goal, we can compute
the goal grid cell as follows (assuming that the `self.global_home` corresponds
to the center of the grid, i.e. the cell: `(-north_offset, -east_offset)`):

```
local_goal = global_to_local(lon_lat_goal, self.global_home)
grid_goal = (
    -north_offset + int(local_goal[0]),
    -east_offset + int(local_goal[1]))
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

I have modified the A* algorithm to run on top of the Voronoi graph constructed
for the given obstacles (where we take the centers of all obstacles as points
for the Voronoi graph). We extract all collision-free edges, i.e. edges not
crossing any `grid` cell with value `1`. We use Bresenham algorithm to find all
grid cells for the given edge. Finally, we trim the resulting path by removing
unnecessary vertices (explained below).

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

For every point `p[i]` in the given path we find the furthermost follow-up
point `p[j]` (with `i < j`), such that all edges `(p[i], p[k])` are
collision-free (for all `i < k <= j`). (We use Bresenham algorithm over the
given 2D `grid` with obstacles to check whether the edge is collision-free.)
We remove all points `p[k]` with `i < k < j`, and keep only the edge
`(p[i], p[j])`. Then we move to the point `p[j]` and repeat the whole process.
We try to iteratively trim the path until we reach a stable state (i.e. a path
that can no longer be trimmed).

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.
