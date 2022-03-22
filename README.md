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

### Build With Docker
```bash
git clone https://github.com/ozakiryota/pc_heightmap_ground_filter.git
cd pc_heightmap_ground_filter/docker
./build.sh
```

## Parameters
* m_per_cell
* grid_dim
* height_diff_threshold

![pc_heightmap_ground_filter](https://user-images.githubusercontent.com/37431972/159501006-4042a5c4-f9ed-419a-8348-179cd0846469.png)

## Usage
1. Edit the launch file
1. Run
```bash
roslaunch pc_heightmap_ground_filter pc_heightmap_ground_filter.launch
```
