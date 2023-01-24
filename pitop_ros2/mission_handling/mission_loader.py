import numpy as np
import yaml


class Waypoint:
    def __init__(self, north: float = 0, east: float = 0):
        self.north = north
        self.east = east


class MissionControl:
    def __init__(self, mission_config, missions_path):
        self.filename = missions_path / mission_config.name
        if not self.filename.exists():
            raise FileNotFoundError(f"Mission file {self.filename} not found")
        data = yaml.safe_load(self.filename.open("r"))
        self.waypoints = np.array(data["waypoints"])
        self.current_waypoint = 0
        self.waypoint_acceptance_radius = mission_config.parameters[
            "waypoint_acceptance_radius"
        ]
        self.loop_waypoints = mission_config.parameters["loop_waypoints"]
        self.finished = False

    def update(self, north, east):
        """If the current position is close to the waypoint go to the next one."""
        diff_x = self.waypoint.north - north
        diff_y = self.waypoint.east - east
        distance = (diff_x ** 2 + diff_y ** 2) ** 0.5
        if distance < self.waypoint_acceptance_radius:
            self.next()

    @property
    def current_waypoint(self) -> Waypoint:
        msg = Waypoint()
        msg.north = self.waypoints[self.current_waypoint, 0]
        msg.east = self.waypoints[self.current_waypoint, 1]
        return msg

    def next(self):
        self.current_waypoint += 1
        if self.current_waypoint >= len(self.waypoints):
            self.finished = not self.loop_waypoints
            self.current_waypoint = 0
