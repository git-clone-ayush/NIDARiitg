import asyncio
import math
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

class DroneController:
    def __init__(self, system_address="serial:///dev/ttyAMA0:57600"):
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
        await self.drone.mission.set_return_to_launch_after_mission(True)
        await self.drone.mission.upload_mission(mission_plan)
        await self.drone.action.arm()
        await self.drone.mission.start_mission()
        async for progress in self.drone.mission.mission_progress():
            print(f"[Mission] Progress: {progress.current}/{progress.total}")
            if progress.current == progress.total:
                print("[DroneController] Mission complete.")
                break
            await asyncio.sleep(1)
    async def land(self):
        await self.drone.action.land()

class MissionPlanner:
    def __init__(self, altitude=3.0, speed=2.0, acceptance_radius=2.0):
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

class Drone:
    def __init__(self):
        self.drone_controller = DroneController()
        self.mission_planner = MissionPlanner()

    async def run(self):
        await self.drone_controller.connect()

        start_pos = await self.drone_controller.get_current_position()
        print(f"[Drone] Current position (start): {start_pos['lat']}, {start_pos['lon']}")

        coord_list = [(start_pos['lat'], start_pos['lon'], MissionItem.VehicleAction.TAKEOFF)]
        print("[INPUT] Enter at least 1 target coordinate (lat lon vehicle_action), one per line. Type 'done' to finish:")
        print("vehicle_action: NONE=0, TAKEOFF=1, LAND=2")

        while True:
            line = await asyncio.to_thread(input, "lat lon action : ")
            if line.strip().lower() == "done":
                break
            try:
                lat, lon, action = map(float, line.strip().split())
                lat = float(lat)
                lon = float(lon)
                action = int(action)
                coord_list.append((lat, lon, MissionItem.VehicleAction(action)))
            except ValueError:
                print("Invalid format. Use: <lat> <lon> <vehicle_action_number>")

        if len(coord_list) < 2:
            print("At least one target coordinate is required after start.")
            return

        mission_items = self.mission_planner.build_from_coords(coord_list)
        mission_plan = MissionPlan(mission_items)

        await self.drone_controller.execute_mission(mission_plan)
        #await self.drone_controller.land()

async def main():
    drone = Drone()
    await drone.run()

if __name__ == "__main__":
    asyncio.run(main())
