import multiprocessing
import numpy as np
import math
import laspy
from multiprocessing import Process, Manager, cpu_count
from RealWorld.handleGeo.ConvCoords import ConvCoords
from RealWorld.real_world_parameter_parser import real_world
import concurrent.futures
from scipy.spatial import cKDTree
from writeToJson import wp_json
import copy

class darp_3d:
    def __init__(self, DARPwaypoints, Hoffset=5, height_margin=0.5, RefineWPS=False) -> None:
        self.DARPWaypoints = DARPwaypoints
        self.real_world_parameters = real_world()
        self.hFOV = np.radians(100)
        self.vFOV = np.radians(83.54)
        self.ODMcoords = self.load_lazfile()
        self.heightOffset = Hoffset
        self.height_margin = height_margin
        self.tolerance = 0.000005 

        # Pre-build a KDTree for fast queries
        self.kd_tree = cKDTree(self.ODMcoords[:, :2])

        # Find reference height and Normalize height values
        self.refHeight = self.find_height(0,0,self.tolerance) 
        self.ODMcoords[:, 2] = -(self.ODMcoords[:, 2] - self.refHeight)

        self.outWaypoints = self.add_height()

        if RefineWPS:
            self.outWaypoints = self.refineWPS()

        self.cameraAngles = self.calcCameraAngles()
        self.writeToJson()


    def load_lazfile(self):
        las = laspy.read('odm/data.laz')

        # Extract x, y, and z coordinates
        x_coords = np.array(las.x)
        y_coords = np.array(las.y)
        z_coords = np.array(las.z)

        print("loaded laz file successfully")

        # add height
        WGS84coords = np.c_[y_coords, x_coords, z_coords]
        return WGS84coords

    def NEDtoWGS(self, x, y):
        return ConvCoords(self.real_world_parameters.geoCoords, self.real_world_parameters.geoObstacles).NEDToWGS84([[[x, y]]])

    def find_height(self, x, y, tolerance):
        # Convert from NED to WGS84
        WGS84coord = self.NEDtoWGS(x,y)
        target_x = WGS84coord[0][0][1]
        target_y = WGS84coord[0][0][0]
        
        # Define the initial tolerance
        current_tolerance = tolerance

        # Use KDTree to find neighbors within tolerance
        while True:
            indices = self.kd_tree.query_ball_point([target_y, target_x], current_tolerance)

            if indices:
                matching_elements = self.ODMcoords[indices]

                # Extract the last column (height values)
                heights = matching_elements[:, -1]

                # Sort the heights in ascending order (more negative is higher)
                sorted_heights = np.sort(heights)

                # Determine the number of points in the top 2% (highest points in the negative direction)
                top_2_percent_count = max(1, int(0.02 * len(sorted_heights))) 

                # Select the top 2% of the highest points (most negative values)
                top_2_percent_heights = sorted_heights[:top_2_percent_count]

                # Compute the mean of the top 2% highest points
                mean_top_2_percent = np.mean(top_2_percent_heights)

                return mean_top_2_percent

            else:
                # Increase tolerance if no matching points found
                current_tolerance += 0.000001

        
    def add_height(self):
        step = 0.01

        manager = Manager()
        shared_height_WP = manager.list()

        def process_drone_waypoints(drone_NO):
            drone_WP = self.DARPWaypoints[drone_NO]
            drone_height_WP = []
            print(f"Started adding height for Drone {drone_NO + 1}")

            for i in range(len(drone_WP)):
                wp_1 = drone_WP[i]
                wp_1_height = self.find_height(wp_1[0], wp_1[1], self.tolerance) - self.heightOffset

                if wp_1_height is None:
                    print("ERROR height for waypoint 1 is None")
                    exit()

                wp_1.append(wp_1_height)
                drone_height_WP.append(wp_1)

                try:
                    wp_2 = drone_WP[i + 1]
                except IndexError:
                    break

                idx = 0 if wp_1[1] == wp_2[1] else 1

                def iterate_points(wp_1, wp_2, idx):
                    prev_height = wp_1[2]
                    iter_step = step if wp_2[idx] > wp_1[idx] else -step

                    for j in np.arange(wp_1[idx], wp_2[idx], iter_step):
                        if idx == 1:
                            height_of_point = self.find_height(wp_1[0], j, self.tolerance) - self.heightOffset
                        else:
                            height_of_point = self.find_height(j, wp_1[1], self.tolerance) - self.heightOffset

                        if height_of_point is None:
                            continue

                        if not math.isclose(height_of_point, prev_height, abs_tol=self.height_margin):
                            waypoint_with_height = [None, None, height_of_point]
                            waypoint_with_height[idx] = j
                            waypoint_with_height[int(not idx)] = wp_1[int(not idx)]

                            drone_height_WP.append(waypoint_with_height)
                            prev_height = height_of_point

                iterate_points(wp_1, wp_2, idx)

            print(f"Finished adding height for Drone {drone_NO + 1}")

            shared_height_WP.append((drone_NO, drone_height_WP))

        processes = []

        for drone_NO in range(len(self.DARPWaypoints)):
            p = Process(target=process_drone_waypoints, args=(drone_NO,))
            processes.append(p)
            p.start()

        for p in processes:
            p.join()

        # Convert shared lists to regular lists for sorting
        height_WP = list(shared_height_WP)

        # Sort results by drone_NO to maintain original order
        height_WP.sort(key=lambda x: x[0])
        height_WP = [item[1] for item in height_WP]

        print('HEIGHT DONE')
        #print(height_WP)
        return height_WP


    def point_exists(self, x, y, z, z_tolerance=0.5):

        # NEDtoWGS
        WGScoords = self.NEDtoWGS(x,y)
        lat = WGScoords[0][0][0]
        lon = WGScoords[0][0][1]

        # Query the KD-Tree for points near (x, y) within xy_tolerance
        indices = self.kd_tree.query_ball_point([lat, lon], self.tolerance)

        # If no points were found within the xy-tolerance, return False
        if len(indices) == 0:
            return False

        # Check the z-coordinate for each matching point in the KD-Tree
        for idx in indices:
            found_z = self.ODMcoords[idx][2]  
            if abs(found_z - z) <= z_tolerance:
                return True

        return False

    def refineWPS(self):

        def adjust_waypoint(current_wp, next_wp, scan_distance):
            def scan_axis(coord_x, coord_y, axis, max_distance):
                offsets = []
                for direction in [1, -1]:
                    for offset in np.arange(0.1, max_distance + 0.1, 0.1):
                        check_x, check_y = (coord_x + offset * direction, coord_y) if axis == 'x' else (coord_x, coord_y + offset * direction)
                        if self.point_exists(check_x, check_y, current_z - 100, 100):
                            offsets.append((direction, offset))
                            break
                return offsets

            current_x, current_y, current_z = current_wp
            adjusted_wp = [current_x, current_y, current_z]

            # Scan in front (primary axis) and behind (opposite direction)
            primary_axis = 'x' if abs(next_wp[0] - current_x) > abs(next_wp[1] - current_y) else 'y'
            secondary_axis = 'y' if primary_axis == 'x' else 'x'

            #print(next_wp[0], current_x, next_wp[1], current_y, next_wp[2], current_z, primary_axis)
            primary_offsets = scan_axis(current_x, current_y, primary_axis, scan_distance)

            def adjust_position(offsets, index):
                if len(offsets) == 1:
                    direction, distance = offsets[0]
                    # Check in the opposite direction before moving
                    opposite_direction = -direction
                    for offset in np.arange(0.1, scan_distance, 0.1):
                        check_pos = (current_x + offset * opposite_direction) if index == 0 else (current_y + offset * opposite_direction)
                        if (self.point_exists(check_pos, current_y, current_z) if index == 0 else self.point_exists(current_x, check_pos, current_z)):
                            # Obstacle found in opposite direction, move to middle
                            middle_pos = (1.2 * direction * distance - opposite_direction * offset) / 2
                            adjusted_wp[index] += middle_pos
                            return True
                    # No obstacle in opposite direction, move fully away
                    adjusted_wp[index] -= direction * abs(0.6 * scan_distance - distance)
                    return True
                elif len(offsets) == 2:
                    pos1 = offsets[0][1] * offsets[0][0]
                    pos2 = offsets[1][1] * offsets[1][0]
                    middle_pos = (pos1 + pos2) / 2
                    adjusted_wp[index] += middle_pos
                    return True
                return False

            # Adjust based on primary axis scans first
            if not adjust_position(primary_offsets, 0 if primary_axis == 'x' else 1):
                # If no adjustment on primary axis, check the secondary axis
                secondary_offsets = scan_axis(current_x, current_y, secondary_axis, scan_distance)
                adjust_position(secondary_offsets, 0 if secondary_axis == 'x' else 1)

            return adjusted_wp

        print("Refining Waypoints")

        scan_distance = self.heightOffset

        def process_drone_waypoints(drone_index, drone_waypoints, scan_distance, output_dict):
            local_waypoints = []
            for i in range(len(drone_waypoints)):
                wp1 = drone_waypoints[i]
                wp2 = drone_waypoints[(i + 1) % len(drone_waypoints)]  # Wraps to the first waypoint when i == len(drone_waypoints) - 1

                new_wp = adjust_waypoint(wp1, wp2, scan_distance)
                local_waypoints.append(new_wp)

            # Store the results in the shared output dictionary
            output_dict[drone_index] = local_waypoints

        def parallel_waypoints(outWaypoints, scan_distance):
            # Create a manager for shared data (output_dict)
            manager = Manager()
            output_dict = manager.dict()

            processes = []

            # Create and start a process for each drone
            for drone_index, drone_waypoints in enumerate(outWaypoints):
                process = Process(
                    target=process_drone_waypoints, 
                    args=(drone_index, drone_waypoints, scan_distance, output_dict)
                )
                processes.append(process)
                process.start()

            # Wait for all processes to complete
            for process in processes:
                process.join()

            # Collect the results from the shared output_dict
            local_waypoints = [output_dict[i] for i in range(len(outWaypoints))]

            return local_waypoints

        final_waypoints = parallel_waypoints(self.outWaypoints, scan_distance)
        print("Refinement Complete")
        return final_waypoints
        
    def fetchPointCircle(self, x, y, z, r):
        pHemisphere = []
        step = 2 

        # Generate all points in spherical coordinates and convert to Cartesian
        points = []
        for theta in np.arange(0, np.pi, step / r):       
            for phi in np.arange(0, 2 * np.pi, step / r):  
                px = x + r * np.sin(theta) * np.cos(phi)
                py = y + r * np.sin(theta) * np.sin(phi)
                pz = z + r * np.cos(theta)  # Positive direction is down

                # Only add points that are within the half-sphere below the drone (positive z direction)
                if pz >= z:
                    points.append((px, py, pz))

        # Query the KD-Tree once for all points to find nearby points
        lat_lon_points = [(self.NEDtoWGS(px, py)[0][0][0], self.NEDtoWGS(px, py)[0][0][1]) for px, py, pz in points]
        indices = self.kd_tree.query_ball_point(lat_lon_points, self.tolerance)

        for i, point in enumerate(points):
            px, py, pz = point
            found_z = None
            # Check the z-coordinate for each point found in the KD-Tree
            for idx in indices[i]:
                lat, lon = lat_lon_points[i]
                found_z = self.ODMcoords[idx][2]  
                if abs(found_z - pz) <= 0.5:  
                    pHemisphere.append([px, py, pz])
                    break

        return pHemisphere


    def calcYaw(self, mPoint, wp1_x, wp1_y, wp2_x, wp2_y):
        
        #highestP = min(pCircle, key=lambda point: point[2])  # airsim: reverse altitude

        yaw_current = math.atan2(wp2_y - wp1_y, wp2_x - wp1_x)

        # Calculate desired yaw angle (direction to the target)
        yaw_desired = math.atan2(mPoint[1] - wp1_y, mPoint[0] - wp1_x)

        # Calculate yaw adjustment needed to face the target
        yaw_adjustment = yaw_desired - yaw_current

        # Normalize yaw_adjustment to be within -π to π
        yaw_adjustment = (yaw_adjustment + math.pi) % (2 * math.pi) - math.pi

        return yaw_adjustment

    def calcPitch(self, mPoint, wp1_x, wp1_y, wp1_z):

        #highestP = min(pCircle, key=lambda point: point[2])  # Select the highest point (lowest altitude in AirSim)

        # Unpack positions
        x_target, y_target, z_target = mPoint

        # Calculate the vertical difference (z-axis) in AirSim's reversed altitude system
        delta_z = -(z_target - wp1_z)  # Invert delta_z to account for airsims reversed altitude

        # Calculate the horizontal distance (in the xy-plane)
        horizontal_distance = math.sqrt((x_target - wp1_x)**2 + (y_target - wp1_y)**2)

        # Calculate the pitch angle (negative pitch = camera tilts down)
        pitch = math.atan2(delta_z, horizontal_distance)

        # Cap the pitch angle to a maximum of 0 degrees (no looking upward)
        pitch = min(pitch, math.radians(0))

        return pitch

    def process_chunk(self, i, chunk_indices):
        """Process a chunk of waypoints for drone i"""
        waypoints = self.outWaypoints[i]
        camera_angles_chunk = []

        for j in chunk_indices:
            wp1 = waypoints[j]
            wp2 = waypoints[j + 1]

            # Fetch point circle logic
            pCircle = None
            for r in np.arange(1, 2 * self.heightOffset, 0.5):
                pCircle = self.fetchPointCircle(wp1[0], wp1[1], wp1[2], r)
                if pCircle:
                    break

            # Angle calculation logic
            if pCircle:
                heights = [point[2] for point in pCircle]
                mean_height = np.mean(heights)
                closest_point = min(pCircle, key=lambda p: abs(p[2] - mean_height))
                yaw = self.calcYaw(closest_point, wp1[0], wp1[1], wp2[0], wp2[1])
                pitch = self.calcPitch(closest_point, wp1[0], wp1[1], wp1[2])
            else:
                yaw = math.radians(0)
                pitch = math.radians(-90)

            camera_angles_chunk.append([yaw, pitch])

        return camera_angles_chunk

    def worker(self, i, camera_angles):
        """Worker that processes one drone using 4 subprocesses"""
        waypoints = self.outWaypoints[i]
        total_j = len(waypoints) - 1

        # Split indices into 4 chunks
        indices = np.arange(total_j)
        chunks = np.array_split(indices, 4)

        # Process chunks in parallel
        with multiprocessing.Pool(4) as pool:
            args = [(i, chunk.tolist()) for chunk in chunks]
            results = pool.starmap(self.process_chunk, args)

        # Combine results from chunks
        combined_cam = []
        
        for res in results:
            combined_cam.extend(res)

        # Wrap around
        if combined_cam:
            combined_cam.append(combined_cam[0])

        camera_angles[i] = combined_cam

    def calcCameraAngles(self):
        print("Calculating Angles")
        nof_drones = len(self.outWaypoints)
        manager = Manager()

        # Shared lists to store results
        camera_angles = manager.list([[] for _ in range(nof_drones)])

        # Start one process per drone (each using 4 subprocesses)
        processes = []
        for i in range(nof_drones):
            p = Process(target=self.worker,
                        args=(i, camera_angles))
            processes.append(p)
            p.start()

        # Wait for completion
        for p in processes:
            p.join()

        print("ANGLES DONE")
        return list(camera_angles)

    def getWaypoints(self):
        return self.outWaypoints

    def getCameraAngles(self):
        return self.cameraAngles

    def writeToJson(self):
        local_wps = copy.deepcopy(self.outWaypoints)

        for drone in local_wps:
            for waypoint in drone:
                waypoint[2] = max(0, -waypoint[2])
        
        wp_json(local_wps, self.cameraAngles)
