import asyncio
import math
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan, VehicleAction



class MissionPlanner:
    def __init__(self, altitude=10.0, speed=5.0, acceptance_radius=2.0):
        self.altitude = altitude
        self.speed = speed
        self.acceptance_radius = acceptance_radius
    def create_mission_item(self, lat, lon, action):
        return MissionItem(
            latitude_deg=lat,
            longitude_deg=lon,
            relative_altitude_m=self.altitude,
            speed_m_s=self.speed,
            is_fly_through=True,  # Or vehicle_action == MissionItem.VehicleAction.NONE
            gimbal_pitch_deg=float('nan'),
            gimbal_yaw_deg=float('nan'),
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=float('nan'),
            camera_photo_interval_s=float('nan'),
            acceptance_radius_m=self.acceptance_radius, 
            yaw_deg=float('nan'),
            camera_photo_distance_m=float('nan'),
            vehicle_action=action # takeoff/land/none
            )
    def build_from_coords(self, coord_list):
        mission_items = []
        for lat, lon, action in coord_list:
            mission_items.append(self.create_mission_item(lat, lon, action))
        return mission_items

