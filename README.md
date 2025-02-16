# DARP-3D

## Description 

This project extends the original [DARP](https://github.com/alice-st/DARP/tree/main) algorithm to generate improved 3D models using a point cloud of the Region of Interest (ROI). It outputs WGS84 waypoints that incorporate height variance and optimized camera angles for better viewpoints. Additionally, the project integrates an [AirSim](https://github.com/microsoft/AirSim) implementation based on [RealWorld2AirSim-DARP](https://github.com/emmarapt/RealWorld2AirSim-DARP) for simulation.

![](https://github.com/konskara/DARP-3D/blob/main/images/DARP3D_paths.png)

## Input/Output:

The algorithm receives as input the following:

- The number of robots/vehicles
- The desired sidelap (sidelap corresponds to the desired ovelap between two sequential images in percentage)
- A polygon Region of Interest (ROI), formatted in WGS84 coordinate system
- A set of obstacles (polygons formatted in WGS84 coordinate system) inside the ROI (optional)
- An initial point cloud of the ROI
- A boolean variable named pathsStrictlyInPoly, to select mode between (paths strictly in poly/better coverage)
- A boolean variable named OptimalInitPos, to select mode between (optimal initial positions True/False in coverage paths)
- The initial positions of the vehicles (optional - if OptimalInitPos is False | Note that the number of the initial positions should always be the same as the number of robots/vehicles)
- The desired percentages for proportional area allocation (optional - if not provided, equal will be used instead | Note that the number of the percentages should always be the same as the number of robots/vehicles and their sum should be 1)

The algorithm outputs a set of waypoints (defining the path) and camera angles (yaw, pitch) for each vehicle in the mission, enabling collaborative coverage of the ROI.

## How to Run

### 1. Clone the Repository
```bash
git clone https://github.com/konskara/DARP-3D.git
cd DARP-3D
```

### 2. Prerequisites:
1. **Configure Input File**  
   - Modify [`inputVariables_choosepath.json`](inputVariables_choosepath.json) with parameters in the order listed above.

2. **Point Cloud Setup**  
   - Place your point cloud file as `./odm/data.laz` in **WGS84 coordinates**.  

> **Note**: WebODM exports point clouds with 0.01 precision, which is insufficient for WGS84.  
> **Workaround**:  
> 1. Use WebODM's UTM output (saved as `data_utm.laz`).  
> 2. Convert to WGS84 using the included pipeline file in `./odm`:  
>    ```bash
>    cd ./odm 
>    pdal pipeline pdal_pipeline.json  
>    ```  
>    This will generate `data.laz` in WGS84 coordinates.

### 3. Set Up Virtual Environment and Install Dependencies
Python 3.9 is required.
```
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 4. Execute the Algorithm 
```
python main.py -Darp3D [options]
```

#### Optional Flags

- **`-Hoffset X`**  
  Adjusts the vertical distance from the terrain by **X** meters.  
  **Default:** `5` meters.

- **`-margin Y`**  
  Modifies the height difference when adding waypoints by **Y** meters. A higher margin results in fewer waypoints (less precision), while a lower margin increases the number of waypoints (more precision).  
  **Default:** `0.5` meters.

- **`-Last`**  
  Executes the last mission.

<!-- ############################################### -->
<!-- LICENSE -->
## License

Distributed under the MIT License.

