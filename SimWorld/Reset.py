# import sys
# airsim_path = '/home/thanos/Documents/AirSim/PythonClient/'
# sys.path.append(airsim_path)

import airsim

# Connect to AirSim
client = airsim.MultirotorClient()

# Arm and disarm function
def arm_and_disarm(drone_name, arm=True):
    if arm:
        client.armDisarm(True, vehicle_name=drone_name)
        print(f"{drone_name} armed.")
    else:
        client.armDisarm(False, vehicle_name=drone_name)
        print(f"{drone_name} disarmed.")

# Arm and disarm each drone
for drone_name in ["Drone1", "Drone2", "Drone3"]:
    # Do something here (e.g., fly the drone)
    arm_and_disarm(drone_name, arm=False)  # Disarm

    client.armDisarm(False, drone_name)

    # that's enough fun for now. let's quit cleanly

client.reset()

for drone_name in ["Drone1", "Drone2", "Drone3"]:
    client.enableApiControl(False, drone_name)

