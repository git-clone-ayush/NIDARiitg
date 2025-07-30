import threading
from rplidar import RPLidar
# === LidarController: handles threaded LiDAR data acquisition ===
class LidarController:
    def __init__(self, port="/dev/tty.usbserial-0001", baudrate=256000):
        self.RPLidar = RPLidar
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        self.latest_scan = []
        self.scan_lock = threading.Lock()
        self.running = False
        self.thread = None
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._collect_lidar_data, daemon=True)
        self.thread.start()
    def get_scan(self):
        with self.scan_lock:
            return self.latest_scan.copy()
    def _collect_lidar_data(self):
        self.lidar = self.RPLidar(self.port, baudrate=self.baudrate)
        temp_scan = []
        d = 2 * 251 * 10  # max range dummy
        last_angle = 0
        try:
            for scan in self.lidar.iter_scans(max_buf_meas=10000):
                for (strength, angle, distance) in scan:
                    if angle > last_angle + 2:
                        i = last_angle + 1
                        while i < angle:
                            temp_scan.append((strength, i, d))
                            i += 0.5
                    temp_scan.append((strength, angle, distance))
                    last_angle = angle

                with self.scan_lock:
                    self.latest_scan = temp_scan.copy()
                temp_scan = []

                if not self.running:
                    break
        except Exception as e:
            print(f"[LidarController] Error: {e}")
        finally:
            self.stop()
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.lidar:
            self.lidar.stop_motor()
            self.lidar.stop()
            self.lidar.disconnect()
