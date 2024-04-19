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


## TODO

- [ ] Change Amiga Model Size to 26 inch width
- [ ] Add tomato plant trees in this model
- [ ] Sensor Fusion ?


## Bugs

- [ ] Rotation Cetner of the Amiga

