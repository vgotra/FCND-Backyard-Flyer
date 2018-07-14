import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT_ARRIVING = 3
    WAYPOINT_ARRIVED = 4
    WAYPOINT_STABILIZATION = 5
    WAYPOINT = 6
    LANDING = 7
    DISARMING = 8


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.expected_local_position = ()
        self.in_mission = True
        self.check_state = {}
        self.default_altitude = 3.0

        # initial state
        self.flight_state = States.MANUAL

        self.register_callback(MsgID.LOCAL_POSITION,
                               self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def are_close(self, col1, col2):
        """This function used to compare values of collections with numeric data
        """
        max_diff = 0.13
        if (len(col1) != len(col2)):
            raise ValueError("Different size of input collections")
        result = []
        for x, y in zip(col1, col2):
            result.append(np.abs(np.abs(x) - np.abs(y)) < max_diff)
        return np.array(result)

    def local_position_callback(self):
        if self.flight_state == States.WAYPOINT_ARRIVING:
            # check are we close to the end of expected location
            completed = self.are_close(
                self.local_position, self.expected_local_position[:3])
            # print("LP: ", self.local_position, ",",
            #      self.expected_local_position[:3], ",", completed)
            # if we arrived - slabilize velocity
            if completed.all():
                print("waypoint arrived")
                self.flight_state = States.WAYPOINT_ARRIVED

    def velocity_callback(self):
        # in real life this data should be in position callback
        # to make smooth flights and easy calculations
        if self.flight_state == States.WAYPOINT_ARRIVED:
            completed = self.are_close(self.local_velocity, [0, 0, 0])
            #print("LV: ", self.local_velocity, ", ", completed)
            # if velocity is stable - check for another waypoint
            if completed.all():
                print("waypoint stabilization")
                self.flight_state = States.WAYPOINT_STABILIZATION

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.TAKEOFF:
            self.calculate_box()
            self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT_ARRIVING:
            pass  # logic in position callback
        elif self.flight_state == States.WAYPOINT_ARRIVED:
            pass  # logic in velocity callback
        elif self.flight_state == States.WAYPOINT_STABILIZATION:
            self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            self.landing_transition()
        elif self.flight_state == States.LANDING:
            self.disarming_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        # using simple square
        self.all_waypoints = [(5, 0, self.default_altitude, 0), (5, 5, self.default_altitude, 0),
                              (0, 5, self.default_altitude, 0), (0, 0, self.default_altitude, 0)]

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        if self.flight_state == States.MANUAL:
            self.set_home_position(
                self.global_position[0], self.global_position[1], self.global_position[2])
            self.arm()
            self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        if self.flight_state == States.ARMING:
            self.target_position[2] = self.default_altitude
            self.takeoff(self.default_altitude)
            self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        # if we have waipoints take first and fly to it
        if len(self.all_waypoints) > 0:
            self.expected_local_position = self.all_waypoints.pop(0)
            self.cmd_position(*self.expected_local_position)
            print("waypoint arriving")
            self.flight_state = States.WAYPOINT_ARRIVING
        else:
            # otherwise - just set that waypoints completed
            # and we should land
            print("waypoint completed")
            self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        if self.flight_state == States.WAYPOINT:
            # make landing smooth
            self.target_position[2] = 0.1
            self.takeoff(0.1)
            self.land()
            self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        if self.flight_state == States.LANDING:
            self.disarm()
            self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        if self.flight_state == States.DISARMING:
            self.release_control()
            self.stop()
            self.in_mission = False
            self.flight_state = States.MANUAL

    def start(self):
        """This method is provided

        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1',
                        help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(
        args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
