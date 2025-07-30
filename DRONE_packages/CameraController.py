import pyrealsense2 as rs
import threading
import queue
import numpy as np
import cv2

#check if height is taken from the center of image 
# === CameraController: Threaded RealSense RGB + Depth frame capture ===
class CameraController:
    def __init__(self, frame_rate=15, resolution=(1280, 720)):
        self.frame_rate = frame_rate
        self.resolution = resolution
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.frame_queue = queue.Queue(maxsize=2)  #can be increased
        self.running = True
        self._camera_thread = None
        self.configure_camera()
        self._start_camera_thread()
    def configure_camera(self):
        width, height = self.resolution
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, self.frame_rate)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, self.frame_rate)
        self.pipeline.start(self.config)
    def _start_camera_thread(self):
        self._camera_thread = threading.Thread(target=self._frame_producer, daemon=True)
        self._camera_thread.start()
    def _frame_producer(self):
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                color = np.asanyarray(color_frame.get_data())
                depth = np.asanyarray(depth_frame.get_data())
                if not self.frame_queue.full():
                    self.frame_queue.put((color, depth))

            except Exception as e:
                print(f"[CameraThread] Error: {e}")
    def capture_frame(self):
        try:
            return self.frame_queue.get(timeout=1.0)  # Returns (color, depth)
        except queue.Empty:
            raise RuntimeError("No frame available in time")
    def display_frame(self, frame, window_name="Drone Surveillance"):
        cv2.imshow(window_name, frame)
        return cv2.waitKey(1) & 0xFF == ord('q')
    def cleanup(self):
        self.running = False
        if self._camera_thread is not None:
            self._camera_thread.join(timeout=2)
        self.pipeline.stop()
        cv2.destroyAllWindows()
   