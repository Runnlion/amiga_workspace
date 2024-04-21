#
# Created on Wed Apr 10 2024
#
# Copyright (c) 2024 NCS-Greenhouse-Group
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#

import rospkg
from pathlib import Path
AMIGA_DESCRIPTION_PKG_PATH = rospkg.RosPack().get_path('amiga_gazebo')
MESHES_FOLDER_PATH = str(Path(AMIGA_DESCRIPTION_PKG_PATH).resolve()) + '/models/'

import rospy
rospy.init_node('level_downloadar',anonymous=False) #run only one script

# Paste the meshes links.
meshes_info = {"farmland_1.zip":['https://drive.usercontent.google.com/download?id=126MiSVtfPnfEBenBNzT2mwYvya0r-Vk_&export=download&authuser=0&confirm=t&uuid=24a3aa35-4d7f-4960-9a3e-7857bdf9d954&at=APZUnTXCA6zO3OOu3LKj3HxJgaeX:1712758042393','a5efad7764a283f82ad5e67d767a5d24'],
               "drc_practice_orange_jersey_barrier.zip":['https://drive.google.com/uc?id=1ZYzXa8ILox5ygAPa_7KC_wAVyjxvqjTs&export=download&confirm=t','f49cb48ae1020bda724ff7ddb19c7a1f'],
               "drc_practice_white_jersey_barrier.zip":['https://drive.google.com/uc?id=1v56bcB5A6Q5ZgmdIGLlpSj1lIcoQtUnA&export=download&confirm=t','f944926dff3366e9289f31fc98dafccf']}


import os
from subprocess import call
rospy.loginfo(f"Checking {len(meshes_info)} items.")
import hashlib
for meshes_file in meshes_info:
    rospy.loginfo(f"Checking File {meshes_file}.")
    if(not os.path.isfile(MESHES_FOLDER_PATH + f"/{meshes_file}")):
        call(["curl", "-L", "-o", 
                MESHES_FOLDER_PATH + f"/{meshes_file}", 
                meshes_info[meshes_file][0]])
        rospy.loginfo("Model Downloaded.")
    else:
        rospy.loginfo(f"Model {meshes_file} exists.")
    md5 = hashlib.md5(open(MESHES_FOLDER_PATH + f"/{meshes_file}",'rb').read()).hexdigest()
    if(str(md5) != meshes_info[meshes_file][1]):
        rospy.logerr(f"Integraity Check Failed. MD5:{str(md5)}")

        quit()
    rospy.loginfo(f"Integraity Check Pass. MD5:{str(md5)}")

import os
import zipfile

def unzip_file(zip_file, extract_to):
    with zipfile.ZipFile(zip_file, 'r') as zip_ref:
        zip_ref.extractall(extract_to)

for meshes_file in meshes_info:
    # Example usage:
    zip_file =  MESHES_FOLDER_PATH + f"/{meshes_file}"  # Specify the path to your zip file
    extract_to = MESHES_FOLDER_PATH  # Specify the directory where you want to extract the files
    rospy.loginfo(f"Extracting file {meshes_file}.")
    unzip_file(zip_file, extract_to)
    rospy.loginfo(f"Deleted file {meshes_file}.")
    os.remove(zip_file)

rospy.signal_shutdown("Program Finished.")