import asyncio
from DroneController import DroneController
class Drone:
    def __init__(self):
        self.drone_controller = DroneController(system_address="udp://:14540")

    async def run(self):
        await self.drone_controller.connect()
        await self.drone_controller.arm()
        await self.drone_controller.takeoff()
        await self.drone_controller.land()
        await asyncio.sleep(5)

async def main():
    drone = Drone()
    await drone.run()

if __name__ == "__main__":
    asyncio.run(main())
