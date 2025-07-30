import math
# === GeoEstimator: Converts image coordinates to geographic locations ===
class GeoEstimator:
    @staticmethod
    async def estimate_object_location(drone, current_pos, x1, y1, x2, y2, img_width, img_height, fov_deg=70.0):
        # Calculate pixel offset from center
        obj_x = (x1 + x2) / 2
        obj_y = (y1 + y2) / 2
        dx_pix = obj_x - img_width / 2
        dy_pix = img_height / 2 - obj_y  # Invert Y

        # Convert FoV to meters-per-pixel
        fov_rad = math.radians(fov_deg)
        altitude = current_pos['alt']
        footprint_width = 2 * altitude * math.tan(fov_rad / 2)
        footprint_height = footprint_width * (img_height / img_width)
        meters_per_pixel_x = footprint_width / img_width
        meters_per_pixel_y = footprint_height / img_height

        dx_m = dx_pix * meters_per_pixel_x
        dy_m = dy_pix * meters_per_pixel_y

        # Get drone heading
        try:
            async for imu in drone.telemetry.heading():
                heading_deg = imu.heading_deg
                break
        except Exception:
            print("[GeoEstimator] Heading unavailable — defaulting to 0°")
            heading_deg = 0.0

        heading_rad = math.radians(heading_deg)

        # Rotate displacement vector to align with drone heading
        dx_rot = dx_m * math.cos(heading_rad) - dy_m * math.sin(heading_rad)
        dy_rot = dx_m * math.sin(heading_rad) + dy_m * math.cos(heading_rad)

        # Convert displacement to latitude and longitude offsets
        dlat = dy_rot / 111320.0
        dlon = dx_rot / (111320.0 * math.cos(math.radians(current_pos['lat'])))
        est_lat = current_pos['lat'] + dlat
        est_lon = current_pos['lon'] + dlon

        return est_lat, est_lon
