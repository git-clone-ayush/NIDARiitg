import asyncio
import threading
from mavsdk import System
from rplidar import RPLidar

MIN_OBSTACLE_DISTANCE = 300  # mm
PORT = "/dev/ttyUSB0"

class DroneController:
    def __init__(self):
        self.drone = System()

    async def connect(self):
        print("[DroneController] Connecting...")
        await self.drone.connect(system_address="serial://dev/ttyAMA0:57600")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("[DroneController] Drone connected.")
                break
        await self._wait_for_gps()

    async def _wait_for_gps(self):
        """print("[DroneController] Waiting for GPS fix...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok:
                print("[DroneController] GPS fix acquired.")
                break
            await asyncio.sleep(1)"""
        print("[DroneController] skipping GPS check for this test")

    async def get_current_position(self):
        async for pos in self.drone.telemetry.position():
            return {
                "lat": pos.latitude_deg,
                "lon": pos.longitude_deg,
                "alt": pos.relative_altitude_m
            }

    """async def wait_for_armable(self):
        print("[DroneController] Waiting until drone is armable...")
        while True:
            async for health in self.drone.telemetry.health():
                if health.is_armable:
                    print("[DroneController] Drone is armable.")
                    return
            await asyncio.sleep(1)"""

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


class LidarController:
    def __init__(self, port=PORT):
        self.lidar = RPLidar(port)
        self.scan_thread = None
        self.scan_lock = threading.Lock()
        self.latest_scan = []
        self.running = False

    def start(self):
        print("[LidarController] Starting lidar scan thread...")
        self.running = True
        self.scan_thread = threading.Thread(target=self._collect_lidar_data)
        self.scan_thread.start()
        print("[LidarController] Thread started.")

    def stop(self):
        self.running = False
        if self.scan_thread:
            self.scan_thread.join()
        self.lidar.stop()
        self.lidar.disconnect()
        print("[LidarController] Stopped.")

    def _collect_lidar_data(self):
        try:
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                temp_scan = []
                for _, angle, distance in scan:
                    if distance > 0:
                        temp_scan.append((angle, distance))
                with self.scan_lock:
                    self.latest_scan = temp_scan.copy()
        except Exception as e:
            print(f"[LidarController] Scan Error: {e}")
            self.running = False

    def get_scan(self):
        with self.scan_lock:
            return self.latest_scan.copy()


class ObstacleAvoider:
    def __init__(self, lidar_controller, min_obstacle_distance=MIN_OBSTACLE_DISTANCE):
        self.lidar = lidar_controller
        self.min_distance = min_obstacle_distance

    def is_obstacle_close(self, scan, fov=30):
        half_fov = fov / 2
        for angle, distance in scan:
            #print("f[ScanData angle={ngle}, dist={distance}]")
            """angle = angle % 360
            if angle <= half_fov or angle >= (360 - half_fov):
                if distance < self.min_distance:
                    print(f"[ObstacleAvoider] Obstacle at {distance:.1f} mm (angle {angle:.1f})")
                    return True"""
            if distance <self.threshold:
                print("f[ObstacleAvoider] Obstaclle at {distance:.1f} mm (angle {angle:.1f})")
                return True
        return False

    async def react_to_obstacle(self, drone_controller):
        scan = self.lidar.get_scan()
        print("f[debug] Lidar scan length: {len(scan)}")
        if self.is_obstacle_close(scan):
            print("[ObstacleAvoider] Obstacle too close! Arming drone.")
            await drone_controller.arm()
            return False
        return True


class Drone:
    def __init__(self):
        self.lidar_controller = LidarController()
        self.obstacle_avoider = ObstacleAvoider(self.lidar_controller)
        self.drone_controller = DroneController()

    async def run(self):
        await self.drone_controller.connect()
        #await self.drone_controller.wait_for_armable()
        self.lidar_controller.start()

        try:
            await self.process_lidar_feed()
        finally:
            self.cleanup()

    async def process_lidar_feed(self):
        try:
            print("[LidarFeed] Lidar feed task started.")
            while True:
                success = await self.obstacle_avoider.react_to_obstacle(self.drone_controller)
                if not success:
                    break
                await asyncio.sleep(0.3)
        except asyncio.CancelledError:
            print("[LidarFeed] Cancelled.")
        except Exception as e:
            print(f"[LidarFeed] Error: {e}")

    def cleanup(self):
        print("[Drone] Cleanup started.")
        self.lidar_controller.stop()
        print("[Drone] Cleanup complete.")


async def main():
    drone = Drone()
    await drone.run()

if __name__ == "__main__":
    asyncio.run(main())
