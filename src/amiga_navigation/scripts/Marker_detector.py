import cv2
from cv_bridge import CvBridge,CvBridgeError

import numpy as np
import rospy
from sensor_msgs.msg import Image
from queue import Queue

class MarkerDetecting:
    def __init__(self) -> None:
        rospy.loginfo("starting marker detecting class")
        self.camera_type = "color" #color
        self.image_queue_color = Queue(maxsize=100) 
        
        if(self.camera_type == "color"):
            self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback, queue_size=1)
            self.frame_id = "camera_color_optical_frame"
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        self.board_corners_C11 = [np.array([[-0.06,0.025,0],[-0.01,0.025,0],[-0.01,-0.025,0],[-0.06,-0.025,0]],dtype=np.float32),]
        self.board_ids_C11 = np.array([[0]], dtype = np.int32)
        self.board_C11 = cv2.aruco.Board(self.board_corners_C11, self.aruco_dict, self.board_ids_C11)
        self.mtx_realsense, self.dist_realsense = self.get_camera_param(1280,720,camera="color")
        self.cv_bridge = CvBridge()
        print(self.get_camera_param)

    
    def color_callback(self, msg:Image)->Image:
        try:
            self.image_queue_color.put(msg) #save data to a queue
            self.get_aruco_pose() #goto aruco detector function
        except CvBridgeError as e:
            print(e)
        return 

    def get_aruco_pose(self):
        # pass
        data = self.image_queue_color.get() #get data(raw)
        self.current_stamp = data.header.stamp
        color_image = self.cv_bridge.imgmsg_to_cv2(data,desired_encoding="passthrough") #trahsform raw data to cv recognizable data
        if(self.camera_type == "color"):
            cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB,color_image)
        gray = self.cv_bridge.imgmsg_to_cv2(data,desired_encoding="passthrough") #trahsform raw data to cv recognizable data
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray,self.aruco_dict,parameters=self.parameters) #get marker
        #defien homo transformation matrix
        transformation = np.zeros((4,4))
        transformation[3,3] = 1
        #define message: is detected and aruco pose
        # is_detected = Bool()
        # is_detected.data = False
        # custon_aruco_array_with_id = Aruco_PoseArray_ID()
        # pose_array = PoseArray() #camera local
        # pose_array.header.frame_id = "camera_color_optical_frame"
        # pose_array.header.stamp = rospy.Time.now()

        # pose_array_base = PoseArray()
        # pose_array_base.header.frame_id = "base"
        # pose_array_base.header.stamp = rospy.Time.now()
        rvec = None
        tvec = None
        print(ids)
        

        # if ids is not None:
        #     self.tool_pose = rospy.wait_for_message("tool_pose", PoseStamped)
        #     T_base_ee = tf.transformations.quaternion_matrix(np.array([self.tool_pose.pose.orientation.x,
        #         self.tool_pose.pose.orientation.y,
        #         self.tool_pose.pose.orientation.z,
        #         self.tool_pose.pose.orientation.w]))
        #     T_base_ee[0,3] = self.tool_pose.pose.position.x
        #     T_base_ee[1,3] = self.tool_pose.pose.position.y
        #     T_base_ee[2,3] = self.tool_pose.pose.position.z

        #     T_base_camera = np.matmul(T_base_ee, self.T_flange_camera)

        #     #Transfrom Flange to Realsense Frame
        #     # print(self.tool_pose)
        #     is_detected.data = True
        #     board_list_indecies = self.get_detected_cube(ids)
        #     rospy.loginfo("Index = %s",str(board_list_indecies))

        #     # print(board_list_indecies)
        #     for index in board_list_indecies:
        #         board = self.boards[index]
        #         pose = Pose()
        #         retval,rvec,tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, self.mtx_realsense, self.dist_realsense, None, None)
        #         Rotation_matrix, jacobian = cv2.Rodrigues(rvec)
        #         transformation[0:3,0:3] = Rotation_matrix
        #         transformation[0:3,3] = tvec[:,0]
        #         # Quaternions ix+jy+kz+w are represented as [x, y, z, w].
        #         quaternion = tf.transformations.quaternion_from_matrix(transformation) #transform rotation matrix(4*4) into quaternion 
        #         pose.orientation.x = quaternion[0]
        #         pose.orientation.y = quaternion[1]
        #         pose.orientation.z = quaternion[2]
        #         pose.orientation.w = quaternion[3]
        #         pose.position.x = transformation[0,3]
        #         pose.position.y = transformation[1,3]
        #         pose.position.z = transformation[2,3]

        #         pose_array.poses.append(pose)
        #         custon_aruco_array_with_id.Aruco_ID.append(index)
        #         pose_base = Pose()
        #         T_base_Marker = np.matmul(T_base_camera , transformation)
        #         quaternion_base_marker = tf.transformations.quaternion_from_matrix(T_base_Marker) #transform rotation matrix(4*4) into quaternion 
        #         pose_base.orientation.x = quaternion_base_marker[0]
        #         pose_base.orientation.y = quaternion_base_marker[1]
        #         pose_base.orientation.z = quaternion_base_marker[2]
        #         pose_base.orientation.w = quaternion_base_marker[3]
        #         pose_base.position.x = T_base_Marker[0,3]
        #         pose_base.position.y = T_base_Marker[1,3]
        #         pose_base.position.z = T_base_Marker[2,3]
        #         pose_array_base.poses.append(pose_base)




            
        #         if(self.draw_axis == True):
        #             cv2.drawFrameAxes(gray, self.mtx_realsense, self.dist_realsense, rvec, tvec, 0.08)
        #             cv2.aruco.drawDetectedMarkers(gray, corners)

        # pose_array.header.frame_id = self.frame_id
        # pose_array.header.stamp = data.header.stamp
        # custon_aruco_array_with_id.Aruco_PoseArray = pose_array
        # custon_aruco_array_with_id.Aruco_Pose_on_Base = pose_array_base
        # self.aruco_pose_arr_pub.publish(custon_aruco_array_with_id)
        # # img = self.cv_bridge.cv2_to_imgmsg(gray,encoding="bgr8")
        # # self.aruco_detection_image_publisher.publish(img)
        # cv2.imshow("frame",gray)
        # cv2.waitKey(1)
        
    def get_camera_param(self,w,h,camera = "color"):
        if camera == "color":
            if (w == 640 and h == 480):
                rospy.logerr("No Calibrtion Data")
                exit()
                mtx = np.array([[0,0,0],[0,0,0],[0,0,0]])
                dist = np.array([0,0,0,0,0])
            elif(w==1280 and h == 720):
                mtx = np.array([[1007.1, 0, 660.3],[0, 1010.8, 353.2],[0.,0.,1.]])
                dist = np.array([[0.2104, -0.4521, 0, 0, 0]])     
            else:
                mtx = np.array([[0,0,0],[0,0,0],[0,0,0]])
                dist = np.array([0,0,0,0,0])
                print("Unsupported Resolution")
        elif camera == "infra1":
            # mtx =  np.array([[1.27957029e+03,0.00000000e+00,6.31331991e+02],[0.00000000e+00,1.28458717e+03,3.75026283e+02],[0.00000000e+00,0.00000000e+00,1.00000000e+00]])
            # dist = np.array([ 0.05931958,4.3991918,-0.04978965,0.0533954,-8.94005432])

            # mtx =  np.array([[1.48608947e+03,0.00000000e+00,6.30727785e+02],[0.00000000e+00,1.66749130e+03,3.68613469e+02],[0.00000000e+00,0.00000000e+00,1.00000000e+00]])
            # dist = np.array([ 4.18364655e-01,5.81069699e+00,6.32639830e-03,1.71190680e-01,-1.34726817e+01])
            # [6.125139933153510e+02,0,0;0,6.124379110895454e+02,0;6.430574411459447e+02,3.646254503624093e+02,1]
            mtx = np.array([[6.136237302316880e+02,0,6.473698539550890e+02],[0,6.136945385959431e+02,3.545153113309425e+02],[0.,0.,1]])
            dist = np.array([[ 0.00387036,-0.01047947,0.00066374,.00108635,.00177674]])
            # mtx = np.array([[0.501,0.801826,0.50205],[0.498772,-0.0558599,0.0639365],[-0.000388256,-0.000769425,-0.0210708]])
        elif camera == "global":
            mtx = np.array([[1.45435845e+03,0,1.03721493e+03],[0,1.45148640e+03,5.06507257e+02],[0.,0.,1]])
            dist = np.array([[-0.04905793,  0.85339218, -0.0062373,   0.01389649, -2.91667807]])
        
        return mtx, dist

rospy.init_node('aruco_detector', anonymous=True)
MD = MarkerDetecting()
rospy.spin()