import asyncio
from mavsdk import System
from mavsdk.action import Action
from mavsdk.offboard import Offboard, VelocityNedYaw, PositionNedYaw
from mavsdk.telemetry import Telemetry
from mavsdk.mission import MissionItem, MissionPlan, MissionProgress
# === ObstacleAvoider: Avoids obstacles using LiDAR + camera confirmation ===
class ObstacleAvoider:
    def __init__(self, lidar_source, min_safe_altitude=1.5, obstacle_area_threshold=5000, min_obstacle_distance=1000):
        self.lidar = lidar_source
        self.MIN_SAFE_ALTITUDE = min_safe_altitude
        self.OBSTACLE_AREA_THRESHOLD = obstacle_area_threshold
        self.MIN_OBSTACLE_DISTANCE = min_obstacle_distance  # in mm
    """def detect_obstacle_ahead(self, lidar_data, fov_angle=30, threshold_distance=5.0):
        # Check if obstacles exist in a forward FOV cone
        for _, angle, distance in lidar_data:
            if abs(angle) <= fov_angle / 2 and distance / 1000 <= threshold_distance:
                return True
        return False """
    def is_obstacle_ahead(self, scan):
        if not scan:
            return False

        for angle, distance in scan:
            if -15 <= angle <= 15 and distance < self.MIN_OBSTACLE_DISTANCE:
                return True
        return False
    async def bypass_obstacle(self, drone, max_attempts=5):
        attempt = 0
        sidestep_directions = [-1.0, 1.0]
        sidestep_attempts = 0
        ascended = False

        while attempt < max_attempts:
            scan = self.lidar.get_scan()
            front_clear = not self.is_obstacle_ahead(scan)

            if front_clear:
                print("[ObstacleAvoider] Path ahead is clear — moving forward.")
                await drone.offboard.set_velocity_ned(VelocityNedYaw(1.0, 0.0, 0.0, 0.0))  # move forward
                await asyncio.sleep(2)

                if await self.is_safe_to_descend(drone):
                    print("[ObstacleAvoider] Safe to descend — returning to original altitude.")
                    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 1.0, 0.0))  # descend
                    await asyncio.sleep(2)
                else:
                    print("[ObstacleAvoider] Cannot descend yet — obstacle still below.")

                return True

            if sidestep_attempts < len(sidestep_directions):
                direction = sidestep_directions[sidestep_attempts]
                print(f"[ObstacleAvoider] Sidestepping {'left' if direction < 0 else 'right'}.")
                await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, direction, 0.0, 0.0))
                sidestep_attempts += 1
            else:
                print("[ObstacleAvoider] Ascending to bypass obstacle.")
                await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -1.0, 0.0))  # ascend
                ascended = True

            await asyncio.sleep(2)
            attempt += 1

        print("[ObstacleAvoider] Failed to bypass obstacle after max attempts.")
        return False

    def is_safe_to_descend(self, depth_frame):
        if depth_frame is None or depth_frame.size == 0:
            print("[ObstacleAvoider] Depth frame invalid.")
            return False

        h, w = depth_frame.shape[:2]
        center_x = w // 2
        center_y = h // 2

        # Define a small sampling window around the center
        patch = depth_frame[center_y-2:center_y+3, center_x-2:center_x+3]
        valid_depths = patch[patch > 0]

        if valid_depths.size == 0:
            print("[ObstacleAvoider] No valid depth at center.")
            return False

        average_depth_mm = np.mean(valid_depths)
        average_depth_m = average_depth_mm / 1000.0

        print(f"[ObstacleAvoider] Estimated ground clearance: {average_depth_m:.2f}m")

        return average_depth_m >= self.MIN_SAFE_ALTITUDE
