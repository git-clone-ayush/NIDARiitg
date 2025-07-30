import cv2
from deep_sort_realtime.deepsort_tracker import DeepSort
import numpy as np
# === CropDetector: Detects and tracks yellow patches using DeepSort ===
class CropDetector:
    def __init__(self, lower_green=(35, 40, 40), upper_green=(85, 255, 255), min_area=300, max_age=30):
        self.lower_green = np.array(lower_green, dtype=np.uint8)
        self.upper_green = np.array(upper_green, dtype=np.uint8)
        self.min_area = min_area
        self.tracker = DeepSort(max_age=max_age)
        #self.tracked_radius = {}
    def detect_yellow_patches(self, color_frame):
        hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)

        # Mask out green → keep yellowish and other non-green areas
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        non_green_mask = cv2.bitwise_not(green_mask)

        # Noise reduction
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        non_green_mask = cv2.morphologyEx(non_green_mask, cv2.MORPH_OPEN, kernel)

        # Contour detection
        contours, _ = cv2.findContours(non_green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_area:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                x1, y1 = int(x - radius), int(y - radius)
                x2, y2 = int(x + radius), int(y + radius)
                detections.append({
                    'bbox': [x1, y1, x2 - x1, y2 - y1],
                    'center': (int(x), int(y)),
                    'radius': float(radius),
                    'confidence': 1.0,
                    'class_name': "yellow_patch"
                })

        return detections
    def update_tracks(self, detections, frame):
        ds_input = []
        meta_info = []

        for det in detections:
            ds_input.append((det['bbox'], det['confidence'], det['class_name']))
            meta_info.append({'center': det['center'],'radius': det['radius']})
    
        tracks = self.tracker.update_tracks(ds_input, frame=frame)

        annotated = frame.copy()
        for i,track in enumerate(tracks):
            if not track.is_confirmed():
                continue
            track_id = track.track_id
            meta=meta_info[i]
            
            cx,cy=meta['center']
            radius=meta['radius']

            #self.tracked_radius[track_id] = radius NOT USED

            # Draw circle and label
            cv2.circle(annotated, (cx, cy), radius, (0, 255, 255), 2)
            cv2.putText(annotated, f"Patch #{track_id}", (cx - 10, cy - radius - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        return annotated, tracks
