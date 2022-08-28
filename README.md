# pc_heightmap_ground_filter

## Demo
![demo](https://user-images.githubusercontent.com/37431972/158047203-0c343126-d70b-4611-a68d-e1d4658c838d.png)

## Installation 
### Build locally
Requirements:
* ROS
* PCL

```bash
cd ~/catkin_ws/src
git clone https://github.com/ozakiryota/pc_heightmap_ground_filter.git
cd ..
catkin_make
```

### Build with Docker
```bash
git clone https://github.com/ozakiryota/pc_heightmap_ground_filter.git
cd pc_heightmap_ground_filter/docker
./build.sh
```

## Parameters
* meter_per_cell
* cell_per_axis
* min_obstacle_height

![pc_heightmap_ground_filter](https://user-images.githubusercontent.com/37431972/187061247-f0dc1d09-1362-427c-b3b8-1156dc0bb65a.png)

## Usage
1. Edit the launch file
1. Run
```bash
roslaunch pc_heightmap_ground_filter pc_heightmap_ground_filter.launch
```
