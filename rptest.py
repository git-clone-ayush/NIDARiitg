import sys
import time
from rplidar import RPLidar

# Change this to the correct COM port for your system
PORT_NAME = '/dev/ttyUSB0'  # Update this based on your Device Manager

def run_lidar():
    lidar = RPLidar(PORT_NAME, baudrate=256000, timeout = 3)
    try:
        
        print("RPLIDAR A3 connected and scanning...")

        for scan in lidar.iter_scans():
            print("---- New Scan ----")
            for (_, angle, distance) in scan:
                print(f"Angle: {angle:.2f}°, Distance: {distance:.2f} mm")
            
            #time.sleep(1)  # Adjust scan interval as needed

    except KeyboardInterrupt:
        print("\nStopping LIDAR...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
     
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("LIDAR disconnected.")

if __name__ == "__main__":
    run_lidar()
