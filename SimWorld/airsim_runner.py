"""Class for utilizing AirSim for Autonomous UAV-based Coverage Path Planning operations"""

import airsim
from SimWorld.simulation_world_parameter_parser import simulation_world
from SimWorld import Image_capture, Set_Camera
from pathlib import Path
from RealWorld.handleGeo.ConvCoords import ConvCoords
import multiprocessing

import numpy as np
from datetime import datetime
import time, math
from math import pi
import subprocess
import threading
import cv2
import os
import json
import copy

def run_airsim(real_world_parameters, init_posNED, WaypointsNED, cameraAngles, Darp3D):

    nof_drones = real_world_parameters.droneNo

    # Define paths
    settings_path = f"{Path.home()}/Documents/AirSim/settings.json" # settings.json path

    # Create a directory to store the images
    current_datetime = datetime.now()
    formatted_datetime = current_datetime.strftime("%Y-%m-%d-%H-%M-%S")
    image_storage_path = f"{Path.home()}/Documents/AirSim/{formatted_datetime}" # image storage path

    # Open the settings and load the data
    with open(settings_path, 'r') as file:
        settings_data = json.load(file)

    # Get Drone names
    drone_names = [name for name in settings_data['Vehicles']]

    # connect to Airsim Drone client
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # Enable API control and arm all drones
    for i in range(nof_drones):
        client.enableApiControl(True, f"{drone_names[i]}")
        client.armDisarm(True, f"{drone_names[i]}")

    # Get predefined parameters
    simulation_world_parameters = simulation_world()
    velocity, distance_threshold, corner_radius = simulation_world_parameters.get_mission_parameters()
    h = - real_world_parameters.altitude

    # Initialize waypoints, image coordinates and initial drone positions as empty global lists
    global path_drone
    global image_coords
    image_coords = [[] for _ in range(nof_drones)]
    path_drone = [[] for _ in range(nof_drones)]
    init_pos_drone = []

    # ------------------ Read path from mCPP ---------------------------------------------------------------------------

    for i in range(nof_drones):
        if Darp3D:
            init_pos_drone.append(airsim.Vector3r(init_posNED[i][0], init_posNED[i][1], init_posNED[i][2]))
        else:
            init_pos_drone.append(airsim.Vector3r(init_posNED[i][0], init_posNED[i][1], -real_world_parameters.altitude))
        #for j in range(len(WaypointsNED[i][0])):
        if Darp3D:
            for j in range(len(WaypointsNED[i])):
                path_drone[i].append(airsim.Vector3r(WaypointsNED[i][j][0], WaypointsNED[i][j][1], WaypointsNED[i][j][2]))
        else:
            for j in range(len(WaypointsNED[i])):
                path_drone[i].append(airsim.Vector3r(WaypointsNED[i][j][0], WaypointsNED[i][j][1], h))
    
    # Last point should be the first point
    
    for i in range(nof_drones):
        last_point = copy.deepcopy(path_drone[i][0])   
        path_drone[i].append(last_point)

    # Drawn lines in AirSim will be placed above actual path so that they don't obstruct image capture
    lines_path = copy.deepcopy(path_drone) # Create deep copy of waypoints for colored line depiction
    init_pos_lines = copy.deepcopy(init_pos_drone) # Create deep copy of initial positions for colored point depiction
#    for i in range(nof_drones):
#        for j in range(len(lines_path[i])):
#            lines_path[i][j].z_val -= 3
#        init_pos_lines[i].z_val -= 3

    client.simFlushPersistentMarkers()

    # Define line strip colors
    colors = [
        [1.0, 2.0, 0, 0],
        [1.0, -1.0, 1, 1],
        [1, 0, 0, 1]
    ]

    # Visualize waypoints
    """ Press T for path visualization"""

    client.simPlotPoints(points=init_pos_lines, size=50, is_persistent=True)
    for i in range(nof_drones):
        if i < len(lines_path):  # Ensure we don't exceed the list length
            client.simPlotLineStrip(points=lines_path[i], color_rgba=colors[i], thickness=30, is_persistent=True)

    # airsim.wait_key('Press any key to takeoff')
    f = []
    for i in range(nof_drones):
        f.append(client.takeoffAsync(vehicle_name=f"{drone_names[i]}"))
    for i in range(nof_drones):
        f[i].join()
    time.sleep(2) # wait a bit until the drone settle

    print("make sure we are hovering at {} meters...".format(-h))
    f_hover = []
    for i in range(nof_drones):
        f_hover.append(client.moveToZAsync(h, 6, vehicle_name=f'Drone{i+1}'))
    for i in range(nof_drones): # wait till previous call is executed and point is reached
        f_hover[i].join()

    # Debug lines
    # image_dir = '/home/thanos/Documents/PERIVALLON/UAV_Dataset_Paper/Datasets/Images/Synthetic/'
    # time.sleep(2) # wait a bit until the drone settle
    # camera_angle = settings_data['CameraDefaults']['Gimbal']['Pitch']
    # for i in range(nof_drones): # wait till previous call is executed and point is reached
    #     get_photo(client, image_dir, drone_names[i], camera_angle)

    time.sleep(2) # wait a bit until the drone settle

    # Move to initial positions
    f_init_pos = []
    for i in range(nof_drones):
        f_init_pos.append(client.moveToPositionAsync(path_drone[i][0].x_val, path_drone[i][0].y_val,
                                                           path_drone[i][0].z_val, velocity, 500,
                                                           airsim.DrivetrainType.ForwardOnly,
                                                           airsim.YawMode(False, 0), 3 + 3 / 2,
                                                           vehicle_name=f'Drone{i+1}'))
    for i in range(nof_drones): # wait till previous call is executed and point is reached
        f_init_pos[i].join()
    time.sleep(2)
    
    for i in range(nof_drones):
        get_time = client.getMultirotorState(f'{drone_names[i]}')

        start_date_time = datetime.fromtimestamp(get_time.timestamp // 1000000000)
        print(f" --- First WP !! --- {drone_names[i]} starts to follow the path at {start_date_time} ---")

    for i in range(nof_drones):
        client.enableApiControl(True, f"Drone{i+1}")

    # Get the offsets of each drone
    d_offsets = []
    d_offsets.append(airsim.Vector3r(settings_data['Vehicles']['Drone1']['X'], settings_data['Vehicles']['Drone1']['Y'], settings_data['Vehicles']['Drone1']['Z']))
    d_offsets.append(airsim.Vector3r(settings_data['Vehicles']['Drone2']['X'], settings_data['Vehicles']['Drone2']['Y'], settings_data['Vehicles']['Drone2']['Z']))
    d_offsets.append(airsim.Vector3r(settings_data['Vehicles']['Drone3']['X'], settings_data['Vehicles']['Drone3']['Y'], settings_data['Vehicles']['Drone3']['Z']))

    # Remove the offsets of each drone so that the path follow is exact
    for i in range(nof_drones):
        for j in range(len(path_drone[i])):
            path_drone[i][j].x_val -= d_offsets[i].x_val
            path_drone[i][j].y_val -= d_offsets[i].y_val
            path_drone[i][j].z_val -= d_offsets[i].z_val

    # Find total path distance and total trip trime for each drone
    total_distances = []
    trip_time = []
    resulting_path = []
    for i in range(nof_drones):
        total_dist = 0
        for j in range(len(path_drone[i])-1):
            dist1 = np.array((path_drone[i][j].x_val, path_drone[i][j].y_val))
            dist2 = np.array((path_drone[i][j+1].x_val, path_drone[i][j+1].y_val))
            dist = np.linalg.norm(dist1 - dist2)
            total_dist += dist
        total_distances.append(total_dist)
        trip_time.append(total_dist/velocity)
    #print(total_distances,trip_time)

    # Send the follow path command for each drone with looking forward and with smooth corner turning and constant velocity
    for i in range(nof_drones):
        resulting_path.append(client.moveOnPathAsync(path_drone[i], velocity, trip_time[i] + 500,
                                                airsim.DrivetrainType.ForwardOnly,
                                                airsim.YawMode(False, 0), 
                                                velocity + (velocity/4),
                                                1, 
                                                vehicle_name=f'Drone{i+1}'))

    # Find total number of images that will be captured for each drone
    #hFOV = real_world_parameters.HFOV
    # Frontlap same as sidelap
    image_height = settings_data['CameraDefaults']['CaptureSettings'][0]['Height']
    image_width = settings_data['CameraDefaults']['CaptureSettings'][0]['Width']
    hFOV = settings_data['CameraDefaults']['CaptureSettings'][0]['FOV_Degrees']
    frontlap = real_world_parameters.sidelap / 100
    aspect_ratio = image_height / image_width
    vFOV = 2 * math.atan(math.tan(hFOV / 2 * math.pi / 180) * aspect_ratio) * 180 / math.pi


    # Ground width,height in 2D (x,y)
    #ground_width =  2 * -h * math.tan(hFOV / 2 * pi / 180)
    ground_height = 2 * -h * math.tan(vFOV / 2 * pi / 180)
    distance_diff = ground_height * (1 - frontlap)
    analogous_dif_value = 0.75
    delay = distance_diff / velocity
    delay *= analogous_dif_value
    nof_images = []
    for i in range(nof_drones):
        nof_images.append(math.ceil(trip_time[i]/delay))

   # if os.path.exists(f"{image_storage_path}/geo.txt"):
   #     os.remove(f"{image_storage_path}/geo.txt")
    os.makedirs(image_storage_path, exist_ok=True) # create image storage folder if it doesn't exist
    with open(f"{image_storage_path}/geo.txt", "a") as file:
        file.write("EPSG:4326\n")

    # remove plotted lines
    if Darp3D:
        client.simFlushPersistentMarkers()

    # Create a list to store events for each drone
    events = [multiprocessing.Event() for _ in range(nof_drones)]

    if Darp3D:
        for i in range(nof_drones):
            p1 = multiprocessing.Process(target=Set_Camera.setCam, args=(path_drone[i], cameraAngles[i], 0.1, drone_names[i], events[i], Darp3D))
            p1.start()
    else:
        for i in range(nof_drones):
            p1 = multiprocessing.Process(target=Set_Camera.setCam, args=(path_drone[i], 0, 0.5, drone_names[i], events[i], Darp3D))
            p1.start()

    # Call image capture script for each drone
    # NOTE: In order for image capture to work you must "poccess" and NOT "eject" in AirSim
    #pid = os.getpid()
    for i in range(nof_drones):
        p2 = multiprocessing.Process(target=Image_capture.capture, args=(path_drone[i], settings_path, image_storage_path, nof_images[i], delay, drone_names[i], events[i]))
        p2.start()
    print("Image capture scheduler has started in the background")

    # Wait until drones fully follow the paths
    for i in range(nof_drones):
        resulting_path[i].join()

        # Signal the subprocesses to stop
        events[i].set()
        print(f'drone{i+1} stop')

#    # Reach last point
#    f_last_pos = []
#    for i in range(nof_drones):
#        f_last_pos.append(client.moveToPositionAsync(path_drone[i][-1].x_val, path_drone[i][-1].y_val,
#                                                        path_drone[i][-1].z_val, velocity, 500,
#                                                        airsim.DrivetrainType.ForwardOnly,
#                                                        airsim.YawMode(False, 0), velocity,
#                                                        vehicle_name=f'Drone{i+1}'))
#    for i in range(nof_drones): # wait till previous call is executed and point is reached
#        f_last_pos[i].join()


    time.sleep(6) # wait a bit more for drones to reach final destination

    # End mission after clicking a button
    # airsim.wait_key('Press any key to reset to original state')
#    for i in range(nof_drones):
#        client.armDisarm(False, "Drone{}".format(i + 1))
#        client.reset()
#        client.enableApiControl(False, "Drone{}".format(i + 1))
    print(
        " -------------- Overall execution time for every drone is written in the Flight_time.txt ------------------- ")
    simulation_world_parameters.file_exec_time.close()
