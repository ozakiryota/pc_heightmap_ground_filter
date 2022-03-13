# pc_heightmap_ground_filter

## Demo
![demo](https://user-images.githubusercontent.com/37431972/158047203-0c343126-d70b-4611-a68d-e1d4658c838d.png)

## Build 
### Locally
Requirements:
* ROS
* PCL

```bash
cd ~/catkin_ws/src
git clone https://github.com/ozakiryota/pc_heightmap_ground_filter.git
cd ..
catkin_make
```

### With Docker
```bash
git clone https://github.com/ozakiryota/pc_heightmap_ground_filter.git
cd pc_heightmap_ground_filter/docker
./build.sh
```

## Usage
1. Edit the launch file
1. Run
```bash
roslaunch pc_heightmap_ground_filter pc_heightmap_ground_filter.launch
```
