import asyncio
from DroneController import DroneController

class Drone:
    def __init__(self):
        self.drone_controller = DroneController(system_address="serial://dev/ttyAMA0:57600")
    async def run(self):
        await self.drone_controller.connect()
        await self.drone_controller.arm()
async def main():
    drone = Drone()
    await drone.run()

if __name__ == "__main__":
    asyncio.run(main())
