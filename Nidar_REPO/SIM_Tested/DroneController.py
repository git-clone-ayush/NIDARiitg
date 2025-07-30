import asyncio
import math
from mavsdk import System

#SYSTEM_ADDRESS="udp://:14540"
#SYSTEM_ADDRESS="serial://dev/ttyAMA0:57600"   for Rasp pi 


class DroneController:
    def __init__(self, system_address):
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

    async def execute_mission(self, mission_plan):
        await self.drone.mission.upload_mission(mission_plan)
        await self.drone.action.arm()
        await self.drone.mission.start_mission()
        async for progress in self.drone.mission.mission_progress():
            if progress.current == progress.total:
                print("[DroneController] Mission complete.")
                break
            await asyncio.sleep(1)

    

    """async def arm_and_takeoff(self):
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        await asyncio.sleep(5)"""
    async def arm(self):
        print("[DroneController] Arming...")
        try:
            await self.drone.action.arm()
            print("[DroneController] Armed.")
        except Exception as e:
            print(f"[DroneController] Arm failed: {e}")

    async def disarm(self):
        print("[DroneController] Disarming...")
        await self.drone.action.disarm()
        print("[DroneController] Disarmed.")

    async def wait_for_armable(self):
        print("[DroneController] Waiting until drone is armable...")
        while True:
            async for health in self.drone.telemetry.health():
                if health.is_armable:
                    print("[DroneController] Drone is armable.")
                    return
            await asyncio.sleep(1)

    async def takeoff(self):
        print(">> Taking off to ~6 feet (~2 meters)...")
        await self.drone.action.set_takeoff_altitude(2.0)
        await self.drone.action.takeoff()
        await asyncio.sleep(5)  # hover for 5 seconds

    async def land(self):
        await self.drone.action.land()
        await asyncio.sleep(5)  # wait for landing to complete

    async def RTL(self):
        print("[DroneController] Return to launch (RTL)...")
        await self.drone.action.return_to_launch()
        print("not ready yet")

