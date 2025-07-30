import numpy as np
import matplotlib.pyplot as plt
import math
from pymavlink import mavutil
# === EnvironmentBoard: Maintains and updates obstacle certainty grid ===
class EnvironmentBoard:
    def __init__(self, board_size=502, center=None):
        self.BOARD_SIZE = board_size
        self.CENTER = center if center is not None else board_size // 2
        self.board = np.zeros((self.BOARD_SIZE, self.BOARD_SIZE), dtype=float)
        self.prev_lat = None
        self.prev_lon = None
        self.prev_alt_interval = None
    def get_board(self):
        return self.board
    def reset_board(self):
        self.board.fill(0)
    def update_from_lidar(self, lidar_data, critical_d=75):
        #update certainity grid based on Lidar reading 
        critical_angles = []
        for _, angle, distance in lidar_data:
            theta = math.radians(angle)
            distance = distance / 10  # mm to cm
            if distance <= critical_d:
                critical_angles.append(angle)
            x = int(self.CENTER + distance * math.cos(theta))
            y = int(self.CENTER + distance * math.sin(theta))

            if 0 <= x < self.BOARD_SIZE and 0 <= y < self.BOARD_SIZE:
                self.board[x, y] += 2
                if self.board[x, y] > 250:
                    self.board[x, y] = 250

            for i in range(1, int(distance)):
                xi = int(self.CENTER + i * math.cos(theta))
                yi = int(self.CENTER + i * math.sin(theta))
                if 0 <= xi < self.BOARD_SIZE and 0 <= yi < self.BOARD_SIZE:
                    self.board[xi, yi] = 0
        return critical_angles
    def gps_to_local(self, lat_curr, lon_curr, lat_prev, lon_prev):
        #Convert GPS displacement to local X, Y displacement in meters
        dy = mavutil.meters_per_deg_lat(lat_curr) * (lat_curr - lat_prev)
        dx = mavutil.meters_per_deg_lon(lat_curr) * (lon_curr - lon_prev)
        return int(round(dx)), int(round(dy))
    def shift_board(self, dx, dy):
        """
        Shifts the board values in the opposite direction of the drone's displacement.
        The displacement vector (dx, dy) is given in grid cell units.
        For example, if the displacement is (dx=+2, dy=+1) [drone moved right 2 cells and up 1 cell],
        then we shift the board values by (-2, -1), so that the obstacles "move" left 2 cells and down 1 cell.
        The function returns a new board with the values shifted.
        """
        shift_x = -int(round(dx))
        shift_y = -int(round(dy))
        new_board = np.zeros_like(self.board) # Create a new board filled with zeros
        # Loop through each cell and shift it if the new indices are within bounds
        for x in range(self.BOARD_SIZE):
            for y in range(self.BOARD_SIZE):
                new_x = x + shift_x
                new_y = y + shift_y
                if 0 <= new_x < self.BOARD_SIZE and 0 <= new_y < self.BOARD_SIZE:
                    new_board[new_x, new_y] = self.board[x, y]
        self.board = new_board
    #CHECK IF shift_board() needs improvement 
    """def shift_board(self, dx, dy):
        self.board = np.roll(self.board, shift=(-dy, -dx), axis=(0, 1))
        # Reset edges to zero
        if dy > 0:
            self.board[:dy, :] = 0
        elif dy < 0:
            self.board[dy:, :] = 0
        if dx > 0:
            self.board[:, :dx] = 0
        elif dx < 0:
            self.board[:, dx:] = 0  """
    async def update_by_gps(self, drone):
        telemetry = await drone.telemetry.position()
        lat_curr, lon_curr = telemetry.latitude_deg, telemetry.longitude_deg
        current_alt = telemetry.relative_altitude_m
        current_alt_interval = int(current_alt / 0.3)

        if current_alt_interval != self.prev_alt_interval:
            print(f"Drone moved to a new altitude interval: {current_alt_interval * 0.3}m")
            self.reset_board()
            self.prev_alt_interval = current_alt_interval
            return self.board

        if self.prev_lat is not None and self.prev_lon is not None:
            dx, dy = self.gps_to_local(lat_curr, lon_curr, self.prev_lat, self.prev_lon)
            self.shift_board(dx, dy)

        self.prev_lat = lat_curr
        self.prev_lon = lon_curr
        self.prev_alt_interval = current_alt_interval
        return self.board
