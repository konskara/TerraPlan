import airsim
import math
import time
import sched
import random

def point_to_segment_distance(px, py, pz, x1, y1, z1, x2, y2, z2):
    # Calculate the squared length of the segment in 3D
    segment_length_squared = (x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2

    if segment_length_squared == 0:
        # Segment is a point, return the distance to the point
        return math.sqrt((px - x1) ** 2 + (py - y1) ** 2 + (pz - z1) ** 2)

    # Projection factor t (clamp between 0 and 1 to stay on the segment)
    t = max(0, min(1, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1) + (pz - z1) * (z2 - z1)) / segment_length_squared))
    
    # Projected point on the segment
    projection_x = x1 + t * (x2 - x1)
    projection_y = y1 + t * (y2 - y1)
    projection_z = z1 + t * (z2 - z1)
    
    # Distance from the point to the projection
    return math.sqrt((px - projection_x) ** 2 + (py - projection_y) ** 2 + (pz - projection_z) ** 2)


def find_closest_segment(x_val, y_val, z_val, waypoints):
    min_distance = float('inf')
    closest_segment = None
    
    for i in range(len(waypoints) - 1):
        x1, y1, z1 = waypoints[i].x_val, waypoints[i].y_val, waypoints[i].z_val
        x2, y2, z2 = waypoints[i+1].x_val, waypoints[i+1].y_val, waypoints[i+1].z_val
        
        dist = point_to_segment_distance(x_val, y_val, z_val, x1, y1, z1, x2, y2, z2)
        if dist < min_distance:
            min_distance = dist
            closest_segment = (i, i + 1)
            wp_index = i 
    
    return wp_index 

def setCam(resulting_path, angles, interval, drone_name, event, Darp3D):

    def _setCam(resulting_path, angles, drone_name):

            client = airsim.MultirotorClient()
            camera_z_offset = 1

            vehicle_position = client.simGetVehiclePose(f"{drone_name}").position
            #vehicle_orientation = client.simGetVehiclePose(f"{drone_name}").orientation
            x_val = vehicle_position.x_val
            y_val = vehicle_position.y_val
            z_val = vehicle_position.z_val
            #vpitch, vroll, vyaw = airsim.to_eularian_angles(vehicle_orientation)

            try:
                if Darp3D:
                    angle_index = find_closest_segment(x_val, y_val, z_val, resulting_path)
                    camera_pose = airsim.Pose(airsim.Vector3r(0, 0, camera_z_offset), airsim.to_quaternion(angles[angle_index][1], 0, angles[angle_index][0]))  #PRY in radians
                else:
                    camera_pose = airsim.Pose(airsim.Vector3r(0, 0, camera_z_offset), airsim.to_quaternion(math.radians(-90), 0, 0))  #PRY in radians

                client.simSetCameraPose(0, camera_pose, f'{drone_name}')
            except:
                pass

    s = sched.scheduler(time.time, time.sleep)
    kwargs = {'resulting_path' : resulting_path, 'angles' : angles, 'drone_name' : drone_name}

    while not event.is_set():
            s.enter(interval, 1, _setCam, kwargs=kwargs)
            s.run()
