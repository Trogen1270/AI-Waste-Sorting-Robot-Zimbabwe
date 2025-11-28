import sys
import cv2
import numpy as np
import time
from ultralytics import YOLO
import math

try:
    import sim
except Exception as e:
    print(f'ERROR: Could not import sim.py: {e}'); sys.exit()

def get_3d_position(client_id, vision_sensor, u, v, resolution):
    res, cam_position = sim.simxGetObjectPosition(client_id, vision_sensor, -1, sim.simx_opmode_blocking)
    if res != sim.simx_return_ok: return None
    angle_deg = 60.0
    KNOWN_CONVEYOR_HEIGHT = 0.895
    distance_to_plane = cam_position[2] - KNOWN_CONVEYOR_HEIGHT
    angle_rad = np.deg2rad(angle_deg)
    view_width_at_plane = 2 * distance_to_plane * np.tan(angle_rad / 2.0)
    m_per_pixel = view_width_at_plane / resolution[0]
    
    world_x = cam_position[0] + (u - resolution[0]/2.0) * m_per_pixel
    world_y = cam_position[1] - (v - resolution[1]/2.0) * m_per_pixel
    world_z = KNOWN_CONVEYOR_HEIGHT
    return [world_x, world_y, world_z]

print("=== Robot Supervisor Script with Custom YOLOv8-OBB Model ===")
model = YOLO('best.pt')
TARGET_CLASS = 'PET bottle'
CONFIDENCE_THRESHOLD = 0.50
client_id = -1

try:
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if client_id != -1:
        print("Connected to CoppeliaSim")
        res, vision_sensor = sim.simxGetObjectHandle(client_id, 'VisionSensor', sim.simx_opmode_blocking)
        sim.simxGetVisionSensorImage(client_id, vision_sensor, 0, sim.simx_opmode_streaming)
        time.sleep(1)
        
        while sim.simxGetConnectionId(client_id) != -1:
            res, object_in_zone = sim.simxGetIntegerSignal(client_id, 'object_in_zone', sim.simx_opmode_blocking)
            if object_in_zone == 1:
                res, resolution, image = sim.simxGetVisionSensorImage(client_id, vision_sensor, 0, sim.simx_opmode_buffer)
                if res == sim.simx_return_ok:
                    image_corrected = [p + 256 if p < 0 else p for p in image]
                    img = np.array(image_corrected, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)
                    img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), 0)
                    
                    results = model(img)
                    if results[0].obb is not None:
                        for box in results[0].obb:
                            if model.names[int(box.cls)] == TARGET_CLASS and box.conf >= CONFIDENCE_THRESHOLD:
                                points = box.xyxyxyxy[0].numpy().astype(int)
                                center_u = int(np.mean(points[:, 0]))
                                center_v = int(np.mean(points[:, 1]))
                                
                                world_pos = get_3d_position(client_id, vision_sensor, center_u, center_v, resolution)
                                if world_pos:
                                    pos_str = f"{world_pos[0]},{world_pos[1]},{world_pos[2]}"
                                    sim.simxSetStringSignal(client_id, "target_coords", pos_str, sim.simx_opmode_oneshot)
                                    sim.simxSetIntegerSignal(client_id, "start_pick", 1, sim.simx_opmode_oneshot)
                                    
                                    # Wait for task completion
                                    while True:
                                        res, task_complete = sim.simxGetIntegerSignal(client_id, 'robot_task_complete', sim.simx_opmode_blocking)
                                        if task_complete == 1: break
                                        time.sleep(0.1)
                else:
                    time.sleep(0.1)
    else:
        print("Failed to connect to CoppeliaSim")
finally:
    if client_id != -1: sim.simxFinish(client_id)
