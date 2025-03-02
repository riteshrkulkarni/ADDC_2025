from dronekit import connect, VehicleMode, LocationGlobalRelative
from picamera2 import Picamera2
import cv2
import time
from pyzbar.pyzbar import decode
from math import radians, cos, sin, asin, sqrt
import numpy as np
from pymavlink import mavutil

# Global Variables
VELOCITY = 3 # m/s
INITIAL_ALTITUDE = 5  # meters
SCAN_ALTITUDE = 2  # meters
GPS_TOLERANCE = 1  # meters
SERVO_CHANNEL = 5  # Pixhawk AUX1

camera = Picamera2()
camera.configure(camera.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
camera.start()

print("Connecting to vehicle...")
vehicle = connect('/dev/ttyAMA0', baud=921600, wait_ready=True)

def haversine(lat1, lon1, lat2, lon2):
    R = 6371
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
    c = 2 * asin(sqrt(a))
    return R * c * 1000

def arm_and_takeoff(altitude):
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
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_alt:.2f} m")
        if current_alt >= altitude * 0.95:
            print("Reached target altitude.")
            break
        time.sleep(1)

def goto_gps_location(lat, lon, alt):
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

def refine_gps_location(lat, lon, alt, step_size=0.0000005):  # ~5-10mm step
    best_coords = (lat, lon)
    for _ in range(3):  # 3 iterations for refinement
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
            if best_distance < 0.01:  # Target 1cm accuracy
                break
    return best_coords

def detect_white_and_align(video_writer):
    for _ in range(50):
        frame = camera.capture_array()
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
    detected_qr_data = None
    detection_start_time = time.time()
    try:
        while True:
            frame = camera.capture_array()
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
        print("helllo")
    return detected_qr_data

def verify_qr_and_drop_payload(stored_qr_data, video_writer):
    print("Scanning for matching QR code at waypoint...")
    match_found = False
    detection_start_time = time.time()
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
        for _ in range(100):
            frame = camera.capture_array()
            cv2.putText(frame, "ADDC 2025", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, "Team aeroKLE", (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            decoded_objects = decode(frame)
            for obj in decoded_objects:
                data = obj.data.decode('utf-8')
                (x, y, w, h) = obj.rect
                color = (0, 255, 0) if data == stored_qr_data else (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(frame, data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                if data == stored_qr_data:
                    print(f"✅ Matching QR Code Detected: {data}")
                    match_found = True
            video_writer.write(frame)
            cv2.imshow('QR Verification', frame)
            if match_found or (cv2.waitKey(1) & 0xFF == ord('q')):
                break
    finally:
        pass
    if match_found:
        print("Descending for payload drop...")
        vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_relative_frame.lat, 
                                                 vehicle.location.global_relative_frame.lon, 0.5))
        time.sleep(5)
        drop_payload()
        return True
    else:
        print("❌ No match found, ascending to 5m...")
        goto_gps_location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, INITIAL_ALTITUDE)
        return False

def drop_payload():
    print("📦 Triggering payload release with servo...")
    vehicle.channels.overrides[SERVO_CHANNEL] = 1500
    time.sleep(0.5)
    vehicle.channels.overrides[SERVO_CHANNEL] = 2000
    time.sleep(1)
    vehicle.channels.overrides[SERVO_CHANNEL] = 1500
    vehicle.channels.overrides.clear()
    print("📦 Payload Dropped!")

def mission():
    gps_coords = [
        (15.367415788589014, 75.12625345575083),
        (15.367379766294292, 75.12610085600696),
        (15.367444496460266, 75.12619948549991),
        (15.367333611269032, 75.1261739977405)
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
            print(f"Refined coordinates: {refined_coords}")
            goto_gps_location(*refined_coords, INITIAL_ALTITUDE)
            if verify_qr_and_drop_payload(stored_qr_data, video_writer):
                print("✅ Payload dropped successfully. Initiating RTL.")
                vehicle.mode = VehicleMode("RTL")
                break
        else:
            print("🚫 No matching QR codes found at any waypoints. Returning to launch.")
            print("Stored QR Data: ", stored_qr_data)
            vehicle.mode = VehicleMode("RTL")
    finally:
        print("\nClosing vehicle connection and cleaning up.")
        video_writer.release()
        camera.stop()
        vehicle.close()

if __name__ == '__main__':
    mission()