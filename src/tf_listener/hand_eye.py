import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation as R

# 从文件加载机械臂末端位姿 (xyzrpy 格式转化为旋转和平移)
def load_robot_poses(file_path):
    R_gripper2base = []
    t_gripper2base = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            x, y, z, roll, pitch, yaw = map(float, line.split())
            # 将 rpy 转换为旋转矩阵
            R_ = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()
            t = np.array([x, y, z])
            R_gripper2base.append(R_)
            t_gripper2base.append(t / 1000) 
    return R_gripper2base, t_gripper2base

# 从图像加载棋盘格角点并计算棋盘格相对于相机的位姿
def load_camera_to_board_poses(image_folder, board_size, square_size):
    R_target2cam = []
    t_target2cam = []
    obj_points = []  # 世界坐标系中的角点
    img_points = []  # 图像中的角点
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[1], 0:board_size[0]].T.reshape(-1, 2) * square_size

    # 遍历图像文件夹
    images = sorted([os.path.join(image_folder, img) for img in os.listdir(image_folder) if img.endswith('.jpg') or img.endswith('.png')])
    for image_path in images:
        img = cv2.imread(image_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, board_size, None)
        if ret:
            # 保存角点
            img_points.append(corners)
            obj_points.append(objp)

            # 计算相机到棋盘的位姿
            ret, rvec, tvec = cv2.solvePnP(objp, corners, camera_matrix, dist_coeffs)
            R, _ = cv2.Rodrigues(rvec)
            R_target2cam.append(R)
            t_target2cam.append(tvec.ravel())
        else:
            print(f"Failed to detect chessboard in {image_path}")

    return R_target2cam, t_target2cam

# 手眼标定
def hand_eye_calibration(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam):
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=cv2.CALIB_HAND_EYE_TSAI
    )
    # 构造 4x4 手眼标定矩阵
    H = np.eye(4)
    H[:3, :3] = R_cam2gripper
    H[:3, 3] = t_cam2gripper.flatten()
    return H

if __name__ == "__main__":
    # 参数配置
    robot_pose_file = "/home/nvidia/tf_ws/src/tf_listener/robot_pose.txt"
    image_folder = "/home/nvidia/tf_ws/src/tf_listener/out_imgs"
    board_size = (6, 9)  # 棋盘格尺寸 (行, 列)
    square_size = 0.027  # 棋盘格每个小方块的边长（单位：米）

    # 假设相机内参和畸变系数（需要根据相机标定结果填写）
    camera_matrix = np.array([[605.0, 0.0, 320.0],
                               [0.0, 605.0, 240.0],
                               [0.0, 0.0, 1.0]])
    dist_coeffs = np.zeros(5)  # 假设无畸变

    # 加载机械臂末端位姿
    R_gripper2base, t_gripper2base = load_robot_poses(robot_pose_file)

    # 加载相机到棋盘的位姿
    R_target2cam, t_target2cam = load_camera_to_board_poses(image_folder, board_size, square_size)

    # 进行手眼标定
    hand_eye_matrix = hand_eye_calibration(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam)
    # final = np.array([[0, 0, 1, 0],
    #                   [1, 0, 0, 0],
    #                   [0, 1, 0, 0],
    #                   [0, 0, 0, 1]]) @ hand_eye_matrix
    print("Hand-Eye Calibration Matrix:\n", hand_eye_matrix)

    