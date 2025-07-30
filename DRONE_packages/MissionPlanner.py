import math
from mavsdk import System
from mavsdk.action import Action
from mavsdk.offboard import Offboard, VelocityNedYaw, PositionNedYaw
from mavsdk.telemetry import Telemetry
from mavsdk.mission import MissionItem, MissionPlan, MissionProgress
from mavsdk.camera import Camera
from mavsdk.param import Param
# === MissionPlanner: Generates and structures survey missions ===
class MissionPlanner:
    def __init__(self, altitude=10.0, speed=5.0, acceptance_radius=2.0):
        self.altitude = altitude
        self.speed = speed
        self.acceptance_radius = acceptance_radius
    def create_mission_item(self, lat, lon, is_takeoff=False, is_landing=False):
        return MissionItem(
            lat,
            lon,
            self.altitude,
            self.speed,
            not (is_takeoff or is_landing),
            float('nan'),
            float('nan'),
            MissionItem.CameraAction.NONE,
            self.acceptance_radius,
            float('nan'),
            float('nan'),
            float('nan')
        )
    def calculate_lawnmower_waypoints(self, start_lat, start_lon, area_width, area_height, cell_size):
        mission_items = []
        x_cells = max(math.ceil(area_width / cell_size), 2)
        y_cells = max(math.ceil(area_height / cell_size), 2)

        lat_per_meter = 1 / 111320.0
        lon_per_meter = 1 / (111320.0 * math.cos(math.radians(start_lat)))

        mission_items.append(self.create_mission_item(start_lat, start_lon, is_takeoff=True))

        for row in range(y_cells):
            lat = start_lat + row * cell_size * lat_per_meter
            if row % 2 == 0:
                for col in range(x_cells):
                    lon = start_lon + col * cell_size * lon_per_meter
                    mission_items.append(self.create_mission_item(lat, lon))
            else:
                for col in range(x_cells - 1, -1, -1):
                    lon = start_lon + col * cell_size * lon_per_meter
                    mission_items.append(self.create_mission_item(lat, lon))

        mission_items.append(self.create_mission_item(start_lat, start_lon, is_landing=True))
        return mission_items, x_cells, y_cells
    def calculate_mission_distance(self, mission_items):
        total_distance = 0.0
        prev_lat, prev_lon = None, None

        for item in mission_items:
            if prev_lat is not None:
                lat_diff = (item.latitude_deg - prev_lat) * 111320.0
                lon_diff = (item.longitude_deg - prev_lon) * 111320.0 * math.cos(math.radians(prev_lat))
                total_distance += math.sqrt(lat_diff**2 + lon_diff**2)
            prev_lat, prev_lon = item.latitude_deg, item.longitude_deg

        return total_distance
