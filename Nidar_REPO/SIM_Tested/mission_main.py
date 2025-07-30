import asyncio
from DroneController import DroneController
from MissionPlanner import MissionPlanner

class Drone:
    def __init__(self):
        self.drone_controller = DroneController()
        self.mission_planner = MissionPlanner()
        print("dont know")
    async def run(self):
        await self.drone_controller.connect()

        start_pos = await self.drone_controller.get_current_position()
        print(f"[NidarDrone] Current position (start): {start_pos['lat']}, {start_pos['lon']}")

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
                coord_list.append((lat, lon, action))
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
