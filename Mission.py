from dronekit import connect, VehicleMode, LocationGlobalRelative
from picamera2 import Picamera2
import cv2
import time
from pyzbar.pyzbar import decode
from math import radians, cos, sin, asin, sqrt
import numpy as np
from pymavlink import mavutil

# Global Variables
VELOCITY = 3  # m/s
INITIAL_ALTITUDE = 5  # meters
SCAN_ALTITUDE = 2  # meters
GPS_TOLERANCE = 1  # meters (initial tolerance, refined later)
SERVO_CHANNEL = 5  # Pixhawk AUX1

# Initialize Picamera2
print("Initializing Picamera2...")
camera = Picamera2()
config = camera.create_video_configuration(main={"size": (640, 480)})
camera.configure(config)
camera.start()
print("Picamera2 started successfully.")

print("Connecting to vehicle...")
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

def haversine(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS points using Haversine formula."""
    R = 6371  # Earth's radius in kilometers
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
    c = 2 * asin(sqrt(a))
    return R * c * 1000  # Convert to meters

def arm_and_takeoff(altitude):
    """Arm the vehicle and take off to specified altitude."""
    print("Arming motors...")
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    print(f"Taking off to {altitude} meters...")
    time.sleep(2)
    vehicle.simple_takeoff(altitude)
    
    start_time = time.time()
    timeout = 15  # 15 seconds timeout
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_alt:.2f} m")
        if current_alt >= altitude * 0.95:
            print("Reached target altitude.")
            break
        if time.time() - start_time > timeout:
            print(f"Timeout of {timeout} seconds reached. Altitude not achieved. Current altitude: {current_alt:.2f} m")
            break
        time.sleep(1)

def goto_gps_location(lat, lon, alt):
    """Navigate to specified GPS coordinates at given altitude."""
    distance = haversine(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, lat, lon)
    estimated_time = distance / VELOCITY
    print(f"Estimated distance: {distance:.2f} m | Time: {estimated_time:.2f} s")
    print(f"Navigating to ({lat}, {lon}) at {alt} meters...")
    target_location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(target_location, groundspeed=VELOCITY)
    while True:
        current = vehicle.location.global_relative_frame
        distance_remaining = haversine(current.lat, current.lon, lat, lon)
        print(f"Current location: ({current.lat:.6f}, {current.lon:.6f}) | Remaining Distance: {distance_remaining:.2f} m")
        if distance_remaining < GPS_TOLERANCE:
            print(f"Reached destination within tolerance: ({lat}, {lon})")
            break
        time.sleep(2)

def refine_gps_location(lat, lon, alt, step_size=0.0000001):  # ~1-2 cm step
    """Refine GPS location with small steps for precision."""
    best_coords = (lat, lon)
    target_accuracy = 0.02  # 2 cm target accuracy in meters
    for _ in range(5):  # Up to 5 iterations for refinement
        offsets = [(step_size, 0), (0, step_size), (-step_size, 0), (0, -step_size)]
        best_distance = float('inf')
        for offset_lat, offset_lon in offsets:
            new_lat = best_coords[0] + offset_lat
            new_lon = best_coords[1] + offset_lon
            goto_gps_location(new_lat, new_lon, alt)
            current = vehicle.location.global_relative_frame
            distance = haversine(current.lat, current.lon, lat, lon)
            if distance < best_distance:
                best_distance = distance
                best_coords = (new_lat, new_lon)
            if best_distance < target_accuracy:
                print(f"Refined to within {best_distance:.4f} m of target ({lat}, {lon})")
                return best_coords
        if best_distance < target_accuracy:
            break
    print(f"Best refinement achieved: {best_distance:.4f} m from target ({lat}, {lon})")
    return best_coords

def detect_white_and_align(video_writer):
    """Detect white color and align drone accordingly."""
    for _ in range(50):
        frame = camera.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        mask = cv2.inRange(frame, np.array([200, 200, 200]), np.array([255, 255, 255]))
        if cv2.countNonZero(mask) > 100:
            print("White color detected, aligning with center...")
            moments = cv2.moments(mask)
            if moments["m00"] != 0:
                cx = int(moments["m10"] / moments["m00"])
                frame_center = 320
                if cx < frame_center - 20 or cx > frame_center + 20:
                    offset_lat = 0.000001 * (cx - frame_center) / 320
                    new_lat = vehicle.location.global_relative_frame.lat + offset_lat
                    new_lon = vehicle.location.global_relative_frame.lon
                    goto_gps_location(new_lat, new_lon, INITIAL_ALTITUDE)
                return True
        video_writer.write(frame)
    print("No white color detected.")
    return False

def detect_and_store_qr(video_writer):
    """Detect and store the initial QR code."""
    detected_qr_data = None
    detection_start_time = time.time()
    try:
        while True:
            frame = camera.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.putText(frame, "ADDC 2025", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, "Team aeroKLE", (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            decoded_objects = decode(frame)
            for obj in decoded_objects:
                data = obj.data.decode('utf-8')
                (x, y, w, h) = obj.rect
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                detected_qr_data = data
                detection_time = time.time() - detection_start_time
                print(f"Initial QR Code Detected and Stored: {data} | Detection Time: {detection_time:.2f} s")
            video_writer.write(frame)
            cv2.imshow('Initial QR Detection', frame)
            if detected_qr_data:
                break
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        print("Closing initial QR detection window...")
        cv2.destroyWindow('Initial QR Detection')
    return detected_qr_data

def verify_qr_and_drop_payload(stored_qr_data, video_writer):
    """Verify QR code at waypoint and drop payload if matched."""
    print(f"Scanning for matching QR code at waypoint... Stored QR: {stored_qr_data}")
    match_found = False
    try:
        square_offsets = [(0.000018, 0), (0, 0.000027), (-0.000018, 0), (0, -0.000027)]
        white_detected = False
        for offset_lat, offset_lon in square_offsets:
            if white_detected:
                break
            current_lat = vehicle.location.global_relative_frame.lat
            current_lon = vehicle.location.global_relative_frame.lon
            new_lat = current_lat + offset_lat
            new_lon = current_lon + offset_lon
            goto_gps_location(new_lat, new_lon, INITIAL_ALTITUDE)
            if detect_white_and_align(video_writer):
                white_detected = True
                break
        if not white_detected:
            print("❌ No white color detected.")
            goto_gps_location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, INITIAL_ALTITUDE)
            return False
        print("Descending to 2m for QR scanning...")
        goto_gps_location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, SCAN_ALTITUDE)
        time.sleep(3)
        print("Opening QR verification window...")
        detected_qr = None
        for _ in range(100):
            frame = camera.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.putText(frame, "ADDC 2025", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, "Team aeroKLE", (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            decoded_objects = decode(frame)
            if not decoded_objects:
                print("No QR code detected in this frame.")
            else:
                for obj in decoded_objects:
                    detected_qr = obj.data.decode('utf-8')
                    (x, y, w, h) = obj.rect
                    color = (0, 255, 0) if detected_qr == stored_qr_data else (0, 0, 255)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(frame, detected_qr, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    if detected_qr == stored_qr_data:
                        print(f"✅ Matching QR Code Detected: {detected_qr}")
                        match_found = True
                    else:
                        print(f"❌ Non-matching QR Code Detected: {detected_qr} (Expected: {stored_qr_data})")
            video_writer.write(frame)
            cv2.imshow('QR Verification', frame)
            if match_found or (cv2.waitKey(1) & 0xFF == ord('q')):
                break
            time.sleep(0.1)
    finally:
        print("Closing QR verification window...")
        cv2.destroyWindow('QR Verification')
    if match_found:
        print("Dropping payload from 2m height...")
        drop_payload()
        print("Ascending to 5m after payload drop...")
        goto_gps_location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, INITIAL_ALTITUDE)
        return True
    else:
        print("❌ No match found, ascending to 5m...")
        goto_gps_location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, INITIAL_ALTITUDE)
        return False

def drop_payload():
    """Trigger payload release using servo."""
    print("📦 Triggering payload release with servo...")
    vehicle.channels.overrides[SERVO_CHANNEL] = 1500
    time.sleep(0.5)
    vehicle.channels.overrides[SERVO_CHANNEL] = 2000
    time.sleep(1)
    vehicle.channels.overrides[SERVO_CHANNEL] = 1500
    vehicle.channels.overrides.clear()
    print("📦 Payload Dropped!")

def mission():
    """Execute the full mission sequence."""
    gps_coords = [
        (-35.3637673, 149.165972),
        (-35.3638192, 149.1671145),
        (-35.3620159, 149.1671146),
        (-35.3620159, 149.165972)
    ]
    video_writer = cv2.VideoWriter('full_mission_feed.avi', cv2.VideoWriter_fourcc(*'XVID'), 20, (640, 480))
    try:
        print("\nStep 1: Detect and store the initial QR code.")
        stored_qr_data = detect_and_store_qr(video_writer)
        time.sleep(5)
        input("Press Enter to arm and takeoff...")
        arm_and_takeoff(INITIAL_ALTITUDE)
        if not stored_qr_data:
            print("⚠️ No initial QR code detected. Aborting mission.")
            return
        
        for i, coord in enumerate(gps_coords):
            print(f"\nNavigating to waypoint {i + 1}...")
            goto_gps_location(*coord, INITIAL_ALTITUDE)
            refined_coords = refine_gps_location(*coord, INITIAL_ALTITUDE)
            goto_gps_location(*refined_coords, INITIAL_ALTITUDE)
            
            print("Scanning for white color...")
            start_time = time.time()
            timeout = 35  # 35 seconds timeout
            while True:
                if detect_white_and_align(video_writer):
                    print("Descending to 2m for QR scanning...")
                    goto_gps_location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, SCAN_ALTITUDE)
                    time.sleep(3)
                    
                    print(f"Verifying QR code... Stored QR: {stored_qr_data}")
                    match_found = False
                    detected_qr = None
                    for _ in range(100):
                        frame = camera.capture_array()
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        cv2.putText(frame, "ADDC 2025", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        cv2.putText(frame, "Team aeroKLE", (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        decoded_objects = decode(frame)
                        if not decoded_objects:
                            print("No QR code detected in this frame.")
                        else:
                            for obj in decoded_objects:
                                detected_qr = obj.data.decode('utf-8')
                                (x, y, w, h) = obj.rect
                                color = (0, 255, 0) if detected_qr == stored_qr_data else (0, 0, 255)
                                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                                cv2.putText(frame, detected_qr, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                                if detected_qr == stored_qr_data:
                                    print(f"✅ Matching QR Code Detected: {detected_qr}")
                                    match_found = True
                                else:
                                    print(f"❌ Non-matching QR Code Detected: {detected_qr} (Expected: {stored_qr_data})")
                        video_writer.write(frame)
                        cv2.imshow('QR Verification', frame)
                        if match_found or (cv2.waitKey(1) & 0xFF == ord('q')):
                            break
                        time.sleep(0.1)
                    
                    if match_found:
                        print("Dropping payload from 2m height...")
                        drop_payload()
                        print("Ascending to 5m after payload drop...")
                        goto_gps_location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, INITIAL_ALTITUDE)
                        print("✅ Payload dropped successfully. Initiating RTL...")
                        vehicle.mode = VehicleMode("RTL")
                        timeout = time.time() + 10
                        while vehicle.mode.name != "RTL" and time.time() < timeout:
                            print(f"Current mode: {vehicle.mode.name}. Waiting for RTL mode...")
                            time.sleep(1)
                        if vehicle.mode.name == "RTL":
                            print("RTL mode successfully engaged.")
                        else:
                            print("⚠️ Failed to switch to RTL mode. Forcing LAND mode as fallback.")
                            vehicle.mode = VehicleMode("LAND")
                            while vehicle.mode.name != "LAND":
                                print("Waiting for LAND mode...")
                                time.sleep(1)
                        return
                    else:
                        print("❌ No match found, ascending to 5m...")
                        goto_gps_location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, INITIAL_ALTITUDE)
                    break
                    
                if time.time() - start_time > timeout:
                    print(f"❌ Timeout of {timeout} seconds reached. No white color detected, moving to next waypoint...")
                    break
                time.sleep(0.1)
        
        print("🚫 No matching QR codes found at any waypoints. Returning to launch.")
        vehicle.mode = VehicleMode("RTL")
        timeout = time.time() + 10
        while vehicle.mode.name != "RTL" and time.time() < timeout:
            print(f"Current mode: {vehicle.mode.name}. Waiting for RTL mode...")
            time.sleep(1)
        if vehicle.mode.name != "RTL":
            print("⚠️ Failed to switch to RTL mode. Forcing LAND mode as fallback.")
            vehicle.mode = VehicleMode("LAND")
            while vehicle.mode.name != "LAND":
                print("Waiting for LAND mode...")
                time.sleep(1)
    finally:
        print("Mission Accomplished\n")
        print("\nClosing vehicle connection and cleaning up.")
        video_writer.release()
        camera.stop()
        cv2.destroyAllWindows()
        vehicle.close()

if __name__ == '__main__':
    mission()