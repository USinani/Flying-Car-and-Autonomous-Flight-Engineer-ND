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
Project Writeup addressing required rubric points follows.


### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

motion_planning.py
has the plan_path() which allows to set the target altitude and safety distance that allow the drone to navigate obstacles in the environmnet.

planning_utils.py has the following functions:
-create_grid() 
will return the minimal north and east coordinates for the grid.
-valid_actions()
returns valid actions based from the gird coordinates and obstacles in the environment
-a_star()
the function that perform the search based on A* algorithm
-heuristic()
returns a numpy array with by considering the current position and the target one
-read_home()
reads the position from the input file and returns the latitude and longitude coordintates
-collinearity_prune()
will prune path point by using collinearity and then return the pruned_path

Here are some image illustration my results

![Top Down view](./misc/)
![Top Down view](./misc/)
![Top Down view](./misc/)
![Top Down view](./misc/)
![Top Down view](./misc/)

Testing and checking the implementation of motion_planning.py and backyard_flyer_solution.py
both scripts implement the similar methods at major scale, though, motion_planning has two distict methods from the other which are:  send_waypoints() and plan_path()

Performance-wise running backyard_flyer_solution.py a square flight path will be generated by the drone whereas when running motion_palnning.py 
the drone will takeoff hit the obstacles and land back to its original position!

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

Below is shown the first line from csv file for lat0 and lon0 floating point values obtain by using self.set_home_position():

lat0 37.792480, lon0 -122.397450
posX,posY,posZ,halfSizeX,halfSizeY,halfSizeZ
-310.2389,-439.2315,85.5,5,5,85.5
-300.2389,-439.2315,85.5,5,5,85.5
-290.2389,-439.2315,85.5,5,5,85.5
-280.2389,-439.2315,85.5,5,5,85.5



#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

The current local position is as follows:

'''
 def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

'''


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position

Setting the grid start position from the local position. 



This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.


### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


