from datetime import datetime
from RealWorld.handleGeo.ConvCoords import ConvCoords
from RealWorld.real_world_parameter_parser import real_world
import math
import json
import time
import random


class wp_json:
    def __init__(self, NEDwps, camAngles) -> None:
        self.base_params = real_world()
        self.converter = ConvCoords(self.base_params.geoCoords, self.base_params.geoObstacles)
        
        # Validate input structure
        if len(NEDwps) != len(camAngles):
            raise ValueError("NEDwps and camAngles must have the same number of drones")
            
        for drone_wps, drone_angles in zip(NEDwps, camAngles):
            if len(drone_wps) != len(drone_angles):
                raise ValueError("Each drone must have matching waypoints and camera angles")

        # Generate unique mission ID and timestamp
        self.mission_id = random.randint(1, 50000)
        self.timestamp = int(time.time() * 1000)
        
        # Initialize mission structure
        self.data = {
            "timestamp": self.timestamp,
            "missionId": self.mission_id,
            "destinationSystem": "dji.phantom.4.pro.hawk.2",
            "sourceSystem": "choosepath-backend",
            "mission": {
                "type": "WAYPOINT_MISSION",
                "speed": 10.0,
                "flightPathMode": "NORMAL",
                "finishedAction": "GO_HOME"
            },
            "waypoints": [],
            "camera": {
                "gimbalPitch": -87.0,  # Default value
                "details": {
                    "type": "SHOOT_PHOTO_WAYPOINTS",
                    "photoWaypoints": []
                }
            }
        }

        self._process_drones(NEDwps, camAngles)
        self._auto_save()

    def _process_drones(self, NEDwps, camAngles):
        """Process all drones and their waypoints"""
        current_id = self.mission_id + 4  # Start ID sequence
        
        for drone_index, (drone_waypoints, drone_angles) in enumerate(zip(NEDwps, camAngles)):
            for wp_index, (ned_point, angles) in enumerate(zip(drone_waypoints, drone_angles)):
                # Convert coordinates for this waypoint
                wgs84 = self.converter.NEDToWGS84([[ned_point]])
                
                # Add waypoint entry
                self._add_waypoint(
                    id=current_id,
                    latitude=wgs84[0][0][0],    
                    longitude=wgs84[0][0][1],  
                    altitude=ned_point[2],
                    drone_index=drone_index,
                    wp_index=wp_index
                )
                
                # Add camera parameters
                self._add_camera_parameters(
                    id=current_id,
                    heading=math.degrees(angles[0]),
                    gimbal_pitch=math.degrees(angles[1])
                )
                
                current_id += 2  # Increment ID by 2 for next waypoint

    def _add_waypoint(self, id: int, latitude: float, longitude: float, altitude: float, 
                     drone_index: int, wp_index: int):
        """Add a waypoint entry with validation"""
        self.data["waypoints"].append({
            "id": id,
            "point": {
                "type": "Point",
                "coordinates": [longitude, latitude]  # GeoJSON order
            },
            "altitude": round(altitude, 2)  # Rounded to 2 decimal places
        })

    def _add_camera_parameters(self, id: int, heading: float, gimbal_pitch: float):
        """Add camera parameters with validation"""
        self.data["camera"]["details"]["photoWaypoints"].append({
            "id": id,
            "rotateAircraft": False,
            "heading": round(heading, 1),  # Rounded to 1 decimal
            "gimbalPitch": round(gimbal_pitch, 1)
        })

    def _auto_save(self):
        """Auto-save with timestamped filename"""
        filename = f'mission_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        self.write_to_json(filename)

    def write_to_json(self, filename: str):
        """Save mission data to JSON file"""
        with open(filename, 'w') as f:
            json.dump(self.data, f, indent=2, ensure_ascii=False)
