from RealWorld.handleGeo.ConvCoords import ConvCoords
from RealWorld.real_world_parameter_parser import real_world
import airsim
import sys, numpy as np, os, cv2
import time
import argparse
import time
import sched
import subprocess
import multiprocessing
import json
from pathlib import Path
from math import pi, radians, degrees
import random

def format_list(images_coords):
        return [[[coord[1], coord[2]] for coord in sublist] for sublist in images_coords]

def get_photo(image_dir, drone_name, image_coords):
    #t1 = time.time()
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    
    # Accessing position components 
    vehicle_position = client.simGetVehiclePose(f"{drone_name}").position
    #vehicle_orientation = client.simGetVehiclePose(f"{drone_name}").orientation
    # Convert quaternion to Euler angles
    #pitch, roll, yaw = airsim.to_eularian_angles(vehicle_orientation)
    camera_z_offset = 1
    x_val = vehicle_position.x_val
    y_val = vehicle_position.y_val
    z_val = vehicle_position.z_val + camera_z_offset
    #print(client.simGetCameraInfo("0"))

    # Making sure we have the same pose of the vehicle 
    #camera_pose = airsim.Pose(airsim.Vector3r(0, 0, camera_z_offset), airsim.to_quaternion(camera_angle*pi/180, 0, yaw))  #PRY in radians
    #client.simSetCameraPose(0, camera_pose, f'{drone_name}')
    camera_orientation = client.simGetCameraInfo("0").pose.orientation
    cpitch, croll, cyaw = airsim.to_eularian_angles(camera_orientation)

    # Image capture and save
    drone_id = drone_name[-1]
    photo = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name=f'{drone_name}')[0]
    image_name = f'{drone_name}_{int(time.time())}'
    filename = os.path.join(image_dir, image_name)
    #print(f"Saving {image_dir}/{image_name}.jpg")
    img1d = np.frombuffer(photo.image_data_uint8, dtype=np.uint8) #get numpy array
    img_rgb = img1d.reshape(photo.height, photo.width, 3) #reshape array to 3 channel image array H X W X 3
    cv2.imwrite(os.path.normpath(filename + '.jpg'), img_rgb) # write to jpg

    image_coords[0].append([image_name +".jpg", x_val, y_val, -z_val, degrees(cyaw), degrees(cpitch), degrees(croll)]) #YPR
    #t2 = time.time()
    #print(f'time took: {t2 - t1}')

def write_geo_txt(real_world_parameters, image_coords, image_storage_path):# Save image coordinates
    image_coordsWGS = ConvCoords(real_world_parameters.geoCoords, real_world_parameters.geoObstacles).NEDToWGS84(format_list(image_coords))
    for sublist_original, sublist_modified in zip(image_coords, image_coordsWGS):
        for index, coord_modified in enumerate(sublist_modified):
            sublist_original[index][1], sublist_original[index][2] = coord_modified[1], coord_modified[0]
    # Open the file in write mode
    with open(f"{image_storage_path}/geo.txt", "a") as file:
        # Write each sublist to the file
        for sublist in image_coords:
            for item in sublist:
                # Convert each item to string and join with spaces
                line = " ".join(str(x) for x in item)
                # Write the line to the file
                file.write(line + "\n")

def capture(resulting_path, settings_path, save_dir, nof_images, delay, drone_name, event):

    # Initial parameters/variables
    curr_image = 0
    image_coords = [[]]
    real_world_parameters = real_world()

    # Open the settings and load the data
    with open(settings_path, 'r') as file:
        settings_data = json.load(file)

    # Begin scheduler
    s = sched.scheduler(time.time, time.sleep)
    kwargs = {'image_dir': save_dir, 'drone_name': drone_name, 'image_coords': image_coords}

    while not event.is_set():
         print(f"{curr_image}/{nof_images}, {drone_name}")
         s.enter(delay, 1, get_photo, kwargs=kwargs)
         s.run() 
         curr_image += 1
    
    write_geo_txt(real_world_parameters, image_coords, save_dir)

