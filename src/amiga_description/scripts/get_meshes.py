#
# Created on Sun Apr 07 2024
#
# Copyright (c) 2024 NC State University
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#

import rospkg
from pathlib import Path
AMIGA_DESCRIPTION_PKG_PATH = rospkg.RosPack().get_path('amiga_description')
MESHES_FOLDER_PATH = str(Path(AMIGA_DESCRIPTION_PKG_PATH).resolve()) + '/meshes'

import rospy
rospy.init_node('meshes_downloadar',anonymous=False) #run only one script

# Paste the meshes links.
meshes_info = {
  'connector.stl':
       ['https://drive.google.com/uc?id=1deKRoKl_ipgYulLzreZNS6dRqQnQkc3S&export=download&confirm=t','5c506130112541e9b630f56090810717'],
   'smartfork.stl':
       ['https://drive.google.com/uc?id=1e8MIVPHHt-yGNmkR-_DlKIkAG3HFsxlS&export=download&confirm=t','5412f5138b339969ce5e0ca88ef822e5'],
   'smartfork_tire.stl':
       ['https://drive.google.com/uc?id=1jzGqbwErwQhHQpjB_5uRVfXanwGZAXM5&export=download&confirm=t','46352e984b952c1b3a3a4cf684d9f40d']
}

import os
from subprocess import call
rospy.loginfo(f"Checking {len(meshes_info)} items.")
import hashlib
for meshes_file in meshes_info:
    rospy.loginfo(f"Checking File {meshes_file}.")
    # if(not os.path.isfile(MESHES_FOLDER_PATH + f"/{meshes_file}")):
    #     call(["curl", "-L", "-o", 
    #             MESHES_FOLDER_PATH + f"/{meshes_file}", 
    #             meshes_info[meshes_file][0]])
    #     rospy.loginfo("Model Downloaded.")
    if not os.path.isfile(MESHES_FOLDER_PATH + f"/{meshes_file}"):
        call(["wget", "-O", 
            MESHES_FOLDER_PATH + f"/{meshes_file}", 
            meshes_info[meshes_file][0]])
        rospy.loginfo("Model Downloaded.")
    else:
        rospy.loginfo(f"Model {meshes_file} exists.")
    md5 = hashlib.md5(open(MESHES_FOLDER_PATH + f"/{meshes_file}",'rb').read()).hexdigest()
    if(str(md5) != meshes_info[meshes_file][1]):
        quit()
    rospy.loginfo(f"Integraity Check Pass. MD5:{str(md5)}")

rospy.signal_shutdown("Program Finished.")