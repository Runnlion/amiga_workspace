#
# Created on Sun Apr 21 2024
#
# Copyright (c) 2024 NCS-Greenhouse-Group
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#

# Given:
#   1. A list of [x1,y1,z1] to [x2,y2,z2]
#   2. Number of plants

# Output:
#   1. A tomato tree model with specific layout
#   2. Automatically Call a subprocess to spawn these plants.


import rospy
import rospkg
import random
import numpy as np
import open3d as o3d
from pathlib import Path
import xml.etree.ElementTree as ET

class Generate_tomato_tree:
    def __init__(self) -> None:
        self.coord_data = {
            'row_1': {'start': [0.941, 2.256, 0.987], 'end': [70.067902, 2.742976, 0.987]},
            'row_2': {'start': [0.941, 5.580445, 0.987], 'end': [70.069817, 5.580445, 0.987]},
            'row_3': {'start': [0.941, 7.968412, 0.987], 'end': [70.069817, 8.442155, 0.987]},
            'row_4': {'start': [-0.084935, 10.85653, 0.987], 'end': [70.069817, 11.323705, 0.987]},
            'row_5': {'start': [-0.409732, 13.593894, 0.987], 'end': [70.069817, 14.258244, 0.987]},
            'row_6': {'start': [-1.331836, 16.492468, 0.987], 'end': [70.069817, 16.996014, 0.987]},
            'row_7': {'start': [-1.499264, 19.287249, 0.987], 'end': [70.069817, 19.924019, 0.987]},
            'row_8': {'start': [-2.250606, 22.132254, 0.987], 'end': [70.069817, 22.527531, 0.987]}
            }
            # Retrieve parameters
        self.row_num:int = rospy.get_param("~row_number", 8)
        self.plant_num:int = rospy.get_param("~plant_num_per_row", 20)
        self.seed_val:int = rospy.get_param("~plant_yaw_seed", 0)
        self.terrain_level:float = rospy.get_param("~terrain_z_offset", 0)
        rospy.loginfo(self.terrain_level)
        # self.plant_num = 20 #default plant number per row
        # self.row_num = 8

        # Configure the random number seed
        # self.seed_val = 0

        random.seed(self.seed_val)

        self.model_name = 'auto_generated_plant_rows'
        self.sdf_name = 'plant_rows'
        self.Amiga_Gazebo_Path = rospkg.RosPack().get_path('amiga_gazebo')
        self.Amiga_Gazebo_Model_Path = str(Path(self.Amiga_Gazebo_Path).resolve()) + '/models/'
        self.obj_file_path = self.Amiga_Gazebo_Model_Path + 'farmland_1/meshes/model.obj'
        self.load_mesh()
        self.z_offset_from_model = 5.0
        self.z_offset_from_manual = self.terrain_level
        self.z_offset_total = self.z_offset_from_model + self.z_offset_from_manual

    def make_directory(self)->None:
        from pathlib import Path

        # Specify the directory path
        self.model_directory_path = Path(f"{self.Amiga_Gazebo_Model_Path}/{self.model_name}_{self.seed_val}")

        # Create the directory
        self.model_directory_path.mkdir(parents=True, exist_ok=True)

    def generate_model_config(self)->None:

        # Create the root element
        model = ET.Element("model")

        # Add sub-elements
        name = ET.SubElement(model, "name")
        name.text = "Plant Rows"

        version = ET.SubElement(model, "version")
        version.text = "1.0"

        sdf = ET.SubElement(model, "sdf")
        sdf.set("version", "1.7")
        sdf.text = f"{self.sdf_name}.sdf"
        
        #? What package should I depend?
        depend = ET.SubElement(model, "depend")
        model_sub = ET.SubElement(depend, "model")
        uri = ET.SubElement(model_sub, "uri")
        uri.text = f"model://{self.model_name}"
        model_version = ET.SubElement(model_sub, "version")
        model_version.text = "1.0"

        description = ET.SubElement(model, "description")
        description.text = f"This is a automatically generated plant rows. Plant Numbers: {self.plant_num} Seed: {self.seed_val}"
        
        import xml.dom.minidom
        # Create the XML tree
        tree = ET.ElementTree(model)

        # Convert XML tree to string
        xml_string = ET.tostring(model, encoding="unicode")

        # Parse the XML string
        parsed_xml = xml.dom.minidom.parseString(xml_string)

        # Pretty print the XML
        pretty_xml = parsed_xml.toprettyxml(indent="  ")

        # Write to file
        with open(f"{self.Amiga_Gazebo_Model_Path}/{self.model_name}_{self.seed_val}/model.config", "w") as file:
            file.write(pretty_xml)

    def generate_plants(self)->None:

        # Create the root element
        self.sdf_file = ET.Element("sdf")
        self.sdf_file.set("version", "1.7")

        # Create the model element
        self.sdf_model = ET.SubElement(self.sdf_file, "model")
        self.sdf_model.set("name", "combined_plants")

        # Add static property
        static = ET.SubElement(self.sdf_model, "static")
        static.text = "true"

        # Add three plant instances
        for row_num in range(1, self.row_num+1):
            row_name = f'row_{row_num}'
            start_pt:list = self.coord_data[row_name]['start']
            end_pt:list = self.coord_data[row_name]['end']
            dx:float = (end_pt[0] - start_pt[0])/(self.plant_num-1)
            dy:float = (end_pt[1] - start_pt[1])/(self.plant_num-1)
            dz:float = 0.0
            for plant_id in range(1, self.plant_num+1):
                yaw = random.uniform(-1.57, 1.57)
                x = start_pt[0] + dx*(plant_id-1)
                y = start_pt[1] + dy*(plant_id-1)
                z = self.find_z_value(x,y)
                rospy.loginfo(f"Estimated Height Plant_{row_num}_{plant_id} = {z}")
                self.add_plant_instance(f"plant_{row_num}_{plant_id}", 
                                        f"{x} {y} {z} 1.57 0 {yaw}")

        import xml.dom.minidom
        # Convert XML tree to string
        xml_string = ET.tostring(self.sdf_file, encoding="unicode")

        # Parse the XML string
        parsed_xml = xml.dom.minidom.parseString(xml_string)

        # Pretty print the XML
        pretty_xml = parsed_xml.toprettyxml(indent="  ")
        # Write to file
        with open(f"{self.Amiga_Gazebo_Model_Path}/{self.model_name}_{self.seed_val}/{self.sdf_name}.sdf", "w") as file:
            file.write(pretty_xml)
    
    # Create a function to add a plant instance
    def add_plant_instance(self, link_name, pose):
        link = ET.SubElement(self.sdf_model, "link")
        link.set("name", link_name)

        # Add static property
        static = ET.SubElement(link, "static")
        static.text = "false"

        gravity = ET.SubElement(link, "gravity")
        gravity.text = "true"

        # Add inertial properties
        inertial = ET.SubElement(link, "inertial")
        inertial_pose = ET.SubElement(inertial, "pose")
        inertial_pose.text = pose
        mass = ET.SubElement(inertial, "mass")
        mass.text = "1.8"
        inertia = ET.SubElement(inertial, "inertia")
        ixx = ET.SubElement(inertia, "ixx")
        ixx.text = "0.03"
        ixy = ET.SubElement(inertia, "ixy")
        ixy.text = "0"
        ixz = ET.SubElement(inertia, "ixz")
        ixz.text = "0"
        iyy = ET.SubElement(inertia, "iyy")
        iyy.text = "0.03"
        iyz = ET.SubElement(inertia, "iyz")
        iyz.text = "0"
        izz = ET.SubElement(inertia, "izz")
        izz.text = "0.03"
        
        # Add collision properties
        collision = ET.SubElement(link, "collision")
        collision.set("name", f"{link_name}_collision")
        collision_pose_elem = ET.SubElement(collision, "pose")
        collision_pose_elem.text = pose
        geometry = ET.SubElement(collision, "geometry")
        mesh = ET.SubElement(geometry, "mesh")
        scale = ET.SubElement(mesh, "scale")
        scale.text = "0.0015 0.0015 0.0015"
        uri = ET.SubElement(mesh, "uri")
        uri.text = "model://plant/meshes/plant.obj"
        
        # Add visual properties
        visual = ET.SubElement(link, "visual")
        visual.set("name", f"{link_name}_visual")
        visual_pose_elem = ET.SubElement(visual, "pose")
        visual_pose_elem.text = pose
        geometry = ET.SubElement(visual, "geometry")
        mesh = ET.SubElement(geometry, "mesh")
        scale = ET.SubElement(mesh, "scale")
        scale.text = "0.0015 0.0015 0.0015"
        uri = ET.SubElement(mesh, "uri")
        uri.text = "model://plant/meshes/plant.obj"
        material = ET.SubElement(visual, "material")
        script = ET.SubElement(material, "script")
        uri = ET.SubElement(script, "uri")
        uri.text = "model://plant/materials/scripts"
        uri = ET.SubElement(script, "uri")
        uri.text = "model://plant/materials/textures"
        name = ET.SubElement(script, "name")
        name.text = "plant/Diffuse"

    def spawn(self):
        import subprocess
        subprocess.call(f"rosrun gazebo_ros spawn_model -file {self.Amiga_Gazebo_Model_Path}/{self.model_name}_{self.seed_val}/{self.sdf_name}.sdf -sdf -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0 -model combained_plants", shell=True)
        
    def find_z_value(self, x, y)->float:
        z = -y
        # Find the nearest vertices
        distances = np.sqrt(np.sum((self.vertices[:, [0, 2]] - [x, z]) ** 2, axis=1))
        nearest_vertex_indices = np.argsort(distances)[:10]  # Consider the three nearest vertices
        # Perform linear interpolation
        nearest_vertices = self.vertices[nearest_vertex_indices]
        z_values = nearest_vertices[:, 1]
        estimated_z = np.mean(z_values) + self.z_offset_total
        return estimated_z 

    def load_mesh(self):
        self.mesh = o3d.io.read_triangle_mesh(self.obj_file_path)
        self.vertices = np.asarray(self.mesh.vertices)
if __name__ == '__main__':
    rospy.init_node('tomato_tree_generating_node')
    gtt = Generate_tomato_tree()
    gtt.make_directory()
    gtt.generate_model_config()
    gtt.generate_plants()
    gtt.spawn()
