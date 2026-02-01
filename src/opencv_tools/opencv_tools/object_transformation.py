import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

class ObjectTransformation:
    def __init__(self):
        # The matrix provided by the user likely represents T_object_aruco (Object to ArUco)
        # because the 0.166m offset is in the second row (Y), but in the user's photo,
        # the marker's X-axis (Red) points towards the keyboard center.
        # Inverting this matrix puts the large offset in the X-axis.
        
        # Original matrix from image (T_object_to_aruco?):
        T_provided = np.array([
            [ 0.0,  1.0,  0.0,  0.024],
            [-1.0,  0.0,  0.0,  0.166],
            [ 0.0,  0.0,  1.0,  0.018],
            [ 0.0,  0.0,  0.0,  1.0]
        ])
        
        # Use the inverse for T_aruco_object
        self.T_aruco_object = np.linalg.inv(T_provided)

    def get_object_pose(self, rvec_aruco, tvec_aruco):
        """
        Calculates the object's rvec and tvec in the camera frame.
        
        Args:
            rvec_aruco: Rotation vector of the ArUco marker (from solvePnP).
            tvec_aruco: Translation vector of the ArUco marker (from solvePnP).
            
        Returns:
            rvec_object, tvec_object: Rotation and translation vectors of the object in the camera frame.
        """
        # Convert ArUco rvec to rotation matrix
        R_camera_aruco, _ = cv2.Rodrigues(rvec_aruco)
        
        # Construct the 4x4 transformation matrix T_camera_aruco
        T_camera_aruco = np.eye(4)
        T_camera_aruco[:3, :3] = R_camera_aruco
        T_camera_aruco[:3, 3] = tvec_aruco.flatten()
        
        # Calculate T_camera_object = T_camera_aruco * T_aruco_object
        T_camera_object = T_camera_aruco @ self.T_aruco_object
        
        # Extract rvec and tvec for the object
        R_camera_object = T_camera_object[:3, :3]
        tvec_object = T_camera_object[:3, 3].reshape((3, 1))
        
        rvec_object, _ = cv2.Rodrigues(R_camera_object)
        
        return rvec_object, tvec_object

    def get_quaternion(self, rvec):
        """Helper to get quaternion from rvec."""
        rot_mat, _ = cv2.Rodrigues(rvec)
        return R.from_matrix(rot_mat).as_quat()
