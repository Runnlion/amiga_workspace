import xml.etree.ElementTree as ET

# Parse the XML file
tree = ET.parse('/mnt/Data/amiga_workspace/src/amiga_gazebo/scripts/farmland_with_barriers.sdf')
root = tree.getroot()

# Iterate through each model element
for model in root.findall('.//model'):
    name = model.get('name')
    pose = model.find('pose').text.split()

    # Extract x, y, z, and yaw values from pose
    x_val, y_val, z_val = map(float, pose[:3])
    yaw_val = float(pose[-1])

    # Check the prefix and generate the command accordingly
    if name.startswith('White_Barrier_'):
        number = name.split('_')[-1]
        command = f'<node name="white_barrier_{number}_spawn_urdf" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" args="-file $(arg sdf_barrier_white) -sdf -x {x_val} -y {y_val} -z 0.85 -R 0 -P 0 -Y {yaw_val} -model white_barrier_{number}"/>'
        print(command)
    elif name.startswith('Orange_Barrier_'):
        number = name.split('_')[-1]
        command = f'<node name="orange_barrier_{number}_spawn_urdf" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" args="-file $(arg sdf_barrier_orange) -sdf -x {x_val} -y {y_val} -z 0.85 -R 0 -P 0 -Y {yaw_val} -model orange_barrier_{number}"/>'
        print(command)
