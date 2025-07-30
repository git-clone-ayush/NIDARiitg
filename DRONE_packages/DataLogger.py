import threading
import queue
import csv
# === DataLogger: Logs patch ID + first appearance GPS only ===
class DataLogger:
    def __init__(self, filename="patch_log.csv"):
        self.filename = filename
        self.logged_ids = {}
        self.queue = queue.Queue()
        self.running = False
        self.worker_thread = None
        self.lock = threading.Lock()
    def initialize(self):
        self.logged_ids.clear()
        self.csv_file = open(self.filename, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["patch_id", "frame_number", "latitude", "longitude"])
        self.running = True
        self.worker_thread = threading.Thread(target=self._log_worker, daemon=True)
        self.worker_thread.start()
    def _log_worker(self):
        while self.running or not self.queue.empty():
            try:
                args = self.queue.get(timeout=1)
                self._write_log(*args)
            except queue.Empty:
                continue
    def _write_log(self, patch_id, frame_number, lat, lon):
        with self.lock:
            if patch_id not in self.logged_ids:
                self.csv_writer.writerow([patch_id, frame_number, f"{lat:.6f}", f"{lon:.6f}"])
                self.csv_file.flush()
                self.logged_ids[patch_id] = frame_number
                print(f"[Logger] Logged patch #{patch_id} at frame {frame_number}, lat:{lat:.6f}, lon:{lon:.6f}")
    def log_patch(self, patch_id, frame_number, lat, lon):
        self.queue.put((patch_id, frame_number, lat, lon))
    def close(self):
        self.running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=2)
        while not self.queue.empty():
            try:
                args = self.queue.get_nowait()
                self._write_log(*args)
            except queue.Empty:
                break
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
