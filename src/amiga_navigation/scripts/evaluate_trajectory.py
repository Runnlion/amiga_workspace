import sys
import rospy
import numpy as np
from PyQt5.QtCore import pyqtSlot
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QFileDialog
from PyQt5.QtCore import QSettings
from scipy.interpolate import interp1d
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from scipy.linalg import orthogonal_procrustes
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        self.setGeometry(300, 300, 300, 200)
        self.setWindowTitle('Trajectory Alignment')

        layout = QVBoxLayout()

        self.listenButton = QPushButton('Start Listen')
        self.listenButton.clicked.connect(self.listenCallback)
        layout.addWidget(self.listenButton)

        self.alignButton = QPushButton('Align')
        self.alignButton.clicked.connect(self.alignCallback)
        layout.addWidget(self.alignButton)

        self.rmseButton = QPushButton('Load_LiDAR_CSV')
        self.rmseButton.clicked.connect(self.lidar_pose_callback)
        layout.addWidget(self.rmseButton)


        self.setLayout(layout)

        self.listening = False
        self.ground_truth_path = Path()
        self.slam_path = Path()
        self.settings = QSettings("YourCompany", "YourApp")
        self.last_directory = self.settings.value("LastDirectory", "/mnt/Data/SLAM_Dataset/")
        self.load_lidar = False

    @pyqtSlot()
    def listenCallback(self):
        if not self.listening:
            self.listenButton.setText('End Listen')
            self.listening = True
            self.ground_truth_path = Path()
            self.slam_path = Path()
            self.gth_subscriber = rospy.Subscriber('/position_ground_truth', Path, self.gtCallback)
            self.slam_result_subscriber = None
            if(self.load_lidar == False):
                self.slam_result_subscriber = rospy.Subscriber('/orb_slam3_ros/trajectory', Path, self.slamCallback)
        else:
            self.listenButton.setText('Start Listen')
            self.listening = False
            self.gth_subscriber.unregister()
            if(self.load_lidar == False):
                self.slam_result_subscriber.unregister()

    def gtCallback(self, msg):
        self.ground_truth_path = msg
        print("Message Received from gtCallback")

    def slamCallback(self, msg):
        self.slam_path = msg
        print("Message Received from slamCallback")

    @pyqtSlot()
    def alignCallback(self):
        # Implement ICP alignment here
        # For demonstration purposes, just print a message
        # Extract trajectories
        if(self.load_lidar == False):
            estimated_trajectory = self.extract_trajectory(self.slam_path)
            estimated_timestamps = np.array([pose.header.stamp.to_sec() for pose in self.slam_path.poses])

        else:
            estimated_trajectory = self.original_lidar_data[:,1:4]
            estimated_timestamps = self.original_lidar_data[:,0]

        ground_truth_trajectory = self.extract_trajectory(self.ground_truth_path)

        # Extract timestamps
        ground_truth_timestamps = np.array([pose.header.stamp.to_sec() for pose in self.ground_truth_path.poses])

        #  = self.interpolate_trajectory(estimated_trajectory, estimated_timestamps, ground_truth_timestamps)
        
        interpolated_trajectory = interp1d(estimated_timestamps, estimated_trajectory, axis=0, kind='linear', fill_value='extrapolate')
        aligned_estimated_trajectory = interpolated_trajectory(ground_truth_timestamps)

        initial_aligned_trajectory = self.align_trajectories(aligned_estimated_trajectory, ground_truth_trajectory)
        
        self.fine_aligned_trajectory = self.icp_refinement(initial_aligned_trajectory, ground_truth_trajectory)

        # Calculate RMSE
        rmse = self.calculate_rmse(self.fine_aligned_trajectory, ground_truth_trajectory)

        self.plot_trajectories(self.fine_aligned_trajectory, ground_truth_trajectory, rmse)
        print(f"RMSE:{rmse}")

    def plot_trajectories(self, estimated_trajectory, ground_truth_trajectory, rmse):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot estimated trajectory
        ax.plot(estimated_trajectory[:, 0], estimated_trajectory[:, 1], estimated_trajectory[:, 2], c='b', label='Estimated Trajectory')
        
        # Plot ground truth trajectory
        ax.plot(ground_truth_trajectory[:, 0], ground_truth_trajectory[:, 1], ground_truth_trajectory[:, 2], c='g', label='Ground Truth Trajectory')
        
        # Labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'3D Trajectories, RMSE: {rmse}')
        ax.legend()
        plt.show()

    def interpolate_trajectory(self, estimated_trajectory, estimated_timestamps, ground_truth_timestamps):
        interpolated_trajectory = interp1d(estimated_timestamps, estimated_trajectory, axis=0, kind='linear', fill_value='extrapolate')
        aligned_trajectory = interpolated_trajectory(ground_truth_timestamps)
        return aligned_trajectory    
    
    def align_trajectories(self, estimated_trajectory, ground_truth_trajectory):
        # Align based on Procrustes analysis
        mu_est = np.mean(estimated_trajectory, axis=0)
        mu_gt = np.mean(ground_truth_trajectory, axis=0)
        
        est_centered = estimated_trajectory - mu_est
        gt_centered = ground_truth_trajectory - mu_gt
        
        R, _ = orthogonal_procrustes(est_centered, gt_centered)
        est_aligned = est_centered @ R + mu_gt
        
        return est_aligned
    

    def icp_refinement(self, source, target):
        source_pcd = o3d.geometry.PointCloud()
        target_pcd = o3d.geometry.PointCloud()
        
        source_pcd.points = o3d.utility.Vector3dVector(source)
        target_pcd.points = o3d.utility.Vector3dVector(target)
        
        threshold = 0.02
        trans_init = np.eye(4)
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source_pcd, target_pcd, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        transformation = reg_p2p.transformation
        source_transformed = np.asarray(source_pcd.transform(transformation).points)
        
        return source_transformed


    def calculate_rmse(self, estimated_trajectory, ground_truth_trajectory):
        assert estimated_trajectory.shape == ground_truth_trajectory.shape, "Trajectories must have the same shape"
        squared_diff = (estimated_trajectory - ground_truth_trajectory) ** 2
        mse = np.mean(squared_diff)
        rmse = np.sqrt(mse)
        return rmse

    @pyqtSlot()
    def lidar_pose_callback(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file, _ = QFileDialog.getOpenFileName(self, "Select CSV File", "", "CSV Files (*.csv)", options=options)
        if file:
            print("Selected file:", file)
            self.original_lidar_data = self.read_csv(file)
            print("CSV Data as NumPy Array:")
            print(self.original_lidar_data)
            self.load_lidar = True

    def read_csv(self, file):
        data = []
        with open(file, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                data.append([float(val) for val in row])  # Convert each value to float
        return np.array(data)


    def extract_trajectory(self,path_msg:Path):
        trajectory = []
        for pose_stamped in path_msg.poses:
            pos:Point = pose_stamped.pose.position
            trajectory.append([pos.x, pos.y, pos.z])
        return np.array(trajectory)
if __name__ == '__main__':
    rospy.init_node("slam_evaluator")
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())