# Amiga Workspace

## Install

```bash=
git clone https://github.com/Runnlion/amiga_workspace.git --recurse-submodules
cd amiga_workspace #Change Dict
catkin_make #Build the Catkin Worksapce

#Add the workspace into bashrc
echo 'source /mnt/Data/amiga_workspace/devel/setup.bash' >> ~/.bashrc

```

## Features

* Amiga Model Definition
* Amiga Controller (Skid-Wheel Comntroller)
* Add gazebo world (farmland) and barriers
![Screenshot from 2024-04-19 10-22-54](https://hackmd.io/_uploads/HJkMuhJbR.png)
* Voledyne LiDAR + Dual-Realsense Camera (Depth)
![Screenshot from 2024-04-19 10-25-50](https://hackmd.io/_uploads/ByL3OnJZR.png)
* IMU Sensor
* GPS Sensor

* 20240421:
    * New Feature: Automatically plant generating with height adjustments. 
    * Optimize the launch file and passthrough more parameters.
    * ![Screenshot from 2024-04-21 11-24-07](https://hackmd.io/_uploads/S12AMYz-0.png)
    * ![Screenshot from 2024-04-21 13-13-30](https://hackmd.io/_uploads/H1he7YG-A.png)

## TODO

- [ ] Change Amiga Model Size to 26 inch width
- [X] Add tomato plant trees in this model
- [ ] Sensor Fusion ?


## Bugs

- [ ] Rotation Cetner of the Amiga

