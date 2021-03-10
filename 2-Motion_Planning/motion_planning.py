# importing libraries
import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
# Importing from udacidrone and planning utils
from planning_utils import a_star, heuristic, create_grid, read_home, collinearity_prune
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, goal_global_position=None):
        super().__init__(connection)
        # posX,  posY,  posZ,   halfSizeX,halfSizeY,halfSizeZ
        # -310.2,-439.2, 85.5, 5, 5, 85.5
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        self.goal_global_position = goal_global_position

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            # compare local position with target position
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition() # when local_position>target_position
        # compare if waypoint position < 4
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 4.0:
                # if more than > 0 waypoints call waypoint transition()
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition() # land with a constant velocity

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        csvdata = msgpack.dumps(self.waypoints)
        self.connection._master.write(csvdata)

# Method to be completed from the seed project
    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a flight path ...")
        TARGET_ALTITUDE = 13
        SAFETY_DISTANCE = 6
        
        # Assign target altitude value to target position []
        self.target_position[2] = TARGET_ALTITUDE
        # relative path of the file
        collidersf = 'colliders.csv'

        # TODO: read lat0, lon0 from colliders into floating point values
        # read first row from collidersf
        csvdata = np.genfromtxt(collidersf, delimiter=',' ,max_rows=1, dtype=None)
        
        lat = csvdata[0]
        lat = lat[5:]
        lat = np.array([lat], dtype='Float64')
        lon = csvdata[1]
        lon = csvdata[5:]
        lon = np.array([lon], dtype='Float64')

        # print data example
        # print("csvdata example", csvdata)
        # csvdata example [b'lat0 37.792480' b' lon0 -122.397450']

        # lat0, lon0 = read_home(collidersf)
        # print(f'Home lat : {lat0}, lon : {lon0}')
        
        # # TODO: set home position to (lat0, lon0, 0)
        self.set_home_position(lon, lat, 0)

        # TODO: retrieve current global position
        glob_position = [self._longitude, self._latitude, self._altitude]

        # local_north, local_east, local_down = global_to_local(self.global_position, self.global_home)
        # display for respective coordinates values
        # print(f'Local => north : {local_north}, east : {local_east}, down : {local_down}')

        # TODO: convert to current local position using global_to_local()
        loc_position = global_to_local(glob_position, self.global_home)

        # glob_position = [self._longitude, self._latitude, self._altitude]
        # global_home, global_position, local_position = global_to_local(glob_position, self.global_home, self.global_position,self.local_position) 
        # display respective values
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        csvdata = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(csvdata, TARGET_ALTITUDE, SAFETY_DISTANCE)
        # display north and east offset data output from create_grid method
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # grid_start_north = int(np.ceil(local_north - north_offset))
        # grid_start_east = int(np.ceil(local_east - east_offset))
        # grid_start tuple int coordinates
        # grid_start = (grid_start_north, grid_start_east)
        
        # TODO: convert start position to current position rather than map center
        grid_start = (int(loc_position[0]-north_offset),int(loc_position[1]-east_offset))
        print("Grid start is : " ,grid_start)
        # Set goal as some arbitrary position on the grid
        # goal_north, goal_east, goal_alt = global_to_local(self.goal_global_position, self.global_home)
        # grid_goal = (int(np.ceil(goal_north - north_offset)), int(np.ceil(goal_east - east_offset)))
        
        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_lon = -121.342321
        goal_lat = 34.766864
        goal_alt = TARGET_ALTITUDE

        global_goal = [goal_lon, goal_lat, goal_alt]
        local_goal = global_to_local(global_goal, self.global_home)
        grid_goal = (int(local_goal[0])-north_offset, int(local_goal[1])-east_offset)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)

        # TODO: prune path to minimize number of waypoints
        # path = collinearity_prune(path)
        pruned_path = self.prune_path(path)
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim
        self.send_waypoints()

    def prune_path(self,path):
        def point(p):
            return np.array([p[0], p[1], 1.]).reshape(1, -1)

        def collinearity_check(p1, p2, p3, epsilon=0.01):
            m = np.concatenate((p1, p2, p3), 0)
            det = np.linalg.det(m)
            return abs(det) < epsilon
            
        pruned_path = []
        #prune the path
        p1 = path[0]
        p2 = path[1]
        # add the start to the pruned path
        pruned_path.append(p1)
        for i in range(2,len(path)):
            p3 = path[i]
            if collinearity_check(point(p1),point(p2),point(p3)):
                p2 = p3
            else:
                pruned_path.append(p2)
                p1 = p2
                p2 = p3
        # add the end to the pruned path
        pruned_path.append(p3)


        return pruned_path


    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--goal_lon', type=str, help="Goal longitude")
    parser.add_argument('--goal_lat', type=str, help="Goal latitude")
    parser.add_argument('--goal_alt', type=str, help="Goal altitude")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    # goal_global_position = np.fromstring(f'{args.goal_lon},{args.goal_lat},{args.goal_alt}', dtype='Float64', sep=',')
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
