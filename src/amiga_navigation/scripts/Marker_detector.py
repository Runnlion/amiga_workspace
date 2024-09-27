import os
import cv2
import rospy
import numpy as np
from queue import Queue
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge,CvBridgeError
import tf.transformations as transformations
class MarkerDetecting:
    def __init__(self) -> None:
        rospy.loginfo("starting marker detecting class")
        self.camera_type = "color" #color
        self.image_queue_color = Queue(maxsize=100) 
        self.draw_axis = True

        if(self.camera_type == "color"):
            self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback, queue_size=1)
            self.frame_id = "camera_color_optical_frame"
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # objPoints[i][0] - left-top point of i-th markerglobal_camera
        # objPoints[i][1] - right-top point of i-th marker
        # objPoints[i][2] - right-bottom point of i-th marker
        # objPoints[i][3] - left-bottom point of i-th marker
        self.board_corners_C11 = np.array([[[0,0,0],[-0.095,0,0],[-0.095,0.095,0],[0,0.095,0]],
                                           [[0,0.105,0],[-0.095,0.105,0],[-0.095,0.2,0],[0,0.2,0]]]
                                           ,dtype=np.float32)
        self.board_ids_C11 = np.array([[0],[1]], dtype = np.int32)
        self.board_C11 = cv2.aruco.Board(self.board_corners_C11, self.aruco_dict, self.board_ids_C11)

        self.board_ids = []
        self.board_ids.append(self.board_ids_C11)
        self.boards = []
        self.boards.append(cv2.aruco.Board(self.board_corners_C11 , self.aruco_dict, self.board_ids_C11))
        self.mtx_realsense, self.dist_realsense = self.get_camera_param(1280,720,camera="color")
        self.cv_bridge = CvBridge()
        self.refindStrategy = True
        self.root_folder = "/home/wolftech/lxiang3.lab/Desktop/sdu6/codes/RAIN-GS/dataset/" + str(rospy.Time.now().to_nsec())
        self.image_folder = self.root_folder + "/images/"
        self.sparse_0_folder = self.root_folder + "/sparse/0/"
        self.camera_txt = self.sparse_0_folder + "cameras.txt"
        self.images_txt = self.sparse_0_folder + "images.txt"

        self.create_folder(self.root_folder)
        self.create_folder(self.image_folder)
        self.create_folder(self.sparse_0_folder)
        
        self.image_index = 0
        self.camera_poses = []

        # print(self.get_camera_param)


    def create_folder(self,directory):
        try:
            print(directory)
            if not os.path.exists(directory):
                os.makedirs(directory)
                print(f"Directory '{directory}' created successfully.")
            else:
                print(f"Directory '{directory}' already exists.")
        except Exception as e:
            print(f"Error creating directory '{directory}': {e}")
    def color_callback(self, msg:Image)->Image:
        try:
            self.image_queue_color.put(msg) #save data to a queue
            self.get_aruco_pose() #goto aruco detector function
        except CvBridgeError as e:
            print(e)
        return 
    def get_detected_cube(self,ids):
        board_list_indecies = []
        for i in range(len(self.board_ids)):
            board_id = self.board_ids[i]
            if(np.any(ids == board_id[0]) or np.any(ids == board_id[1])):
                board_list_indecies.append(i)
        return board_list_indecies
    
    def get_aruco_pose(self):
        # pass
        data = self.image_queue_color.get() #get data(raw)
        self.current_stamp = data.header.stamp
        color_image_original = self.cv_bridge.imgmsg_to_cv2(data,desired_encoding="passthrough") #trahsform raw data to cv recognizable data
        gray = self.cv_bridge.imgmsg_to_cv2(data,desired_encoding="passthrough") #trahsform raw data to cv recognizable data
        if(self.camera_type == "color"):
            # pass
            gray = cv2.cvtColor(gray, cv2.COLOR_RGB2BGR)
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray) #get marker
        # result, rvecs, tvecs = cv2.solvePnP(self.board_corners_C11[0], corners[0], self.mtx_realsense, self.dist_realsense)
        

        
        if ids is not None:
            # print(ids)
            board_list_indecies = self.get_detected_cube(ids)
            rospy.loginfo("Board Index = %s",str(board_list_indecies))

            for index in board_list_indecies:
                board:cv2.aruco.Board = self.boards[index]
                if(self.refindStrategy):
                    self.detector.refineDetectedMarkers(gray, board, corners, ids, rejectedImgPoints, self.mtx_realsense,
                                           self.dist_realsense)
                    objPoints, imgPoints =board.matchImagePoints(corners,ids)
                    result = cv2.solvePnP(objPoints,imgPoints,self.mtx_realsense,self.dist_realsense)                    
                    if(result[0]):
                        self.image_index += 1
                        cv2.drawFrameAxes(gray,self.mtx_realsense,self.dist_realsense,result[1],result[2],0.1)
                        rospy.loginfo(f"Translation: {result[2].flatten()}, Rotation: {result[1].flatten()}.")
                        # I should save the image and its pose
                        # The image should be in the images folder and the pose should input to the images.txt, also append the 
                        cv2.imwrite(self.image_folder + str(self.image_index).zfill(4) + ".jpg",cv2.cvtColor(color_image_original,cv2.COLOR_BGR2RGB))
                        R, _ = cv2.Rodrigues(result[1])
                        rotation_matrix_4x4 = np.eye(4)
                        rotation_matrix_4x4[:3, :3] = R
                        qx, qy, qz, qw = transformations.quaternion_from_matrix(rotation_matrix_4x4)
                        tx, ty, tz = result[2].flatten()
                        self.camera_poses.append([qw, qx, qy, qz, tx, ty, tz])
                        print(self.camera_poses[0])
        
        cv2.imshow("frame",gray)
        cv2.waitKey(1)
        
    def get_camera_param(self,w,h,camera = "color"):
        if camera == "color":
            if(w==1280 and h == 720):
                mtx = np.array([[934.48293786, 0, 655.67678338],[ 0.0, 938.50308959, 331.48645769],[0.,0.,1.]])
                # dist = np.array([[0.2104, -0.4521, 0, 0, 0]])
                dist = np.array([[0.1684071,  -0.31638624, -0.01630709,  0.00089548,  0.11028751]])     
            else:
                mtx = np.array([[0,0,0],[0,0,0],[0,0,0]])
                dist = np.array([0,0,0,0,0])
                print("Unsupported Resolution")
        elif camera == "infra1":
            mtx = np.array([[6.136237302316880e+02,0,6.473698539550890e+02],[0,6.136945385959431e+02,3.545153113309425e+02],[0.,0.,1]])
            dist = np.array([[ 0.00387036,-0.01047947,0.00066374,.00108635,.00177674]])
            # mtx = np.array([[0.501,0.801826,0.50205],[0.498772,-0.0558599,0.0639365],[-0.000388256,-0.000769425,-0.0210708]])
        return mtx, dist

    def save_cameratxt(self):
        with open(self.camera_txt,'w') as f:
            for i in range(1, self.image_index + 1):
                colmap_format = f"{i} PINHOLE 1280 720 934.48293786 938.50308959 655.67678338 938.50308959"
                f.write(colmap_format)
                f.write('\n')
            print(f"Camera data saved to {self.camera_txt}")
        with open(self.images_txt,'w') as f:
            for i in range(1, self.image_index + 1):
                data = self.camera_poses[i-1]
                f.write(f"{i} {data[0]} {data[1]} {data[2]} {data[3]} {data[4]} {data[5]} {data[6]} {i} {str(i).zfill(4)}.jpg")
                f.write("\n")
                f.write("\n")
            print(f"Image data saved to {self.images_txt}")


rospy.init_node('aruco_detector', anonymous=True)
MD = MarkerDetecting()
rospy.spin()

while (not rospy.is_shutdown()):
    rospy.Rate(10).sleep()
else:
    MD.save_cameratxt()