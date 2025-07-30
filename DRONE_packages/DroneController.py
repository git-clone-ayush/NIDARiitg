import asyncio
import math
from mavsdk import System
from mavsdk.action import Action
from mavsdk.offboard import Offboard, VelocityNedYaw, PositionNedYaw
from mavsdk.telemetry import Telemetry
from mavsdk.mission import MissionItem, MissionPlan, MissionProgress
from mavsdk.camera import Camera
from mavsdk.param import Param
# === DroneController: Connects, commands, and controls the drone ===
#NEED TO UPDATE TO MAKE COMPATIBLE FOR CUSTOM MISSION PLANNER
class DroneController:
    def __init__(self, system_address="udp://:14540"):
        self.drone = System()
        self.system_address = system_address
    async def connect(self):
        await self.drone.connect(system_address=self.system_address)
        print("[DroneController] Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("[DroneController] Drone connected!")
                break

        print("[DroneController] Waiting for global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok:
                print("[DroneController] GPS lock acquired.")
                break
    async def get_current_position(self):
        async for position in self.drone.telemetry.position():
            if not (math.isnan(position.latitude_deg) or math.isnan(position.longitude_deg)):
                return {
                    'lat': position.latitude_deg,
                    'lon': position.longitude_deg,
                    'alt': position.absolute_altitude_m
                }
            raise ValueError("No valid GPS position received")
    async def execute_mission(self, mission_items):
        mission_plan = MissionPlan(mission_items)  # Create MissionPlan
        await self.drone.mission.upload_mission(mission_plan)
        await self.drone.action.arm()
        await self.drone.mission.start_mission()
        async for progress in self.drone.mission.mission_progress():
            if progress.current == progress.total:
                print("[DroneController] Mission complete.")
                break
            await asyncio.sleep(1)
    async def land(self):
        await self.drone.action.land()
