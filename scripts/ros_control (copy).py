#!/usr/bin/env python3
import argparse
from copy import deepcopy
import time
import numpy as np
from sensor_msgs.msg import Image as IMG
import rospy
from mecanum_control.msg import MecanumCommand

from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as RR



class Car_Controller:
    def __init__(self):
        rospy.init_node('control_car')
        rospy.Subscriber("/zd/pose1", PoseStamped, self.zd_pose1_callback)
        rospy.Subscriber("/zd/pose2", PoseStamped, self.zd_pose2_callback) ## source target
        self.cmd_pub = rospy.Publisher('/mecanum_cmd', MecanumCommand, queue_size=10)


    def pose_to_matrix(self, pose):
        """
        将位姿 (x, y, z, qx, qy, qz, qw) 转换为 4x4 的齐次变换矩阵
        :param pose: 位姿，形状为 (7,)
        :return: 4x4 的齐次变换矩阵
        """
        x, y, z, qx, qy, qz, qw = pose
        r = RR.from_quat(np.array([qx, qy, qz, qw]))
        mat = r.as_matrix()
        T_mat = np.identity(4)
        T_mat[:3, :3] = mat
        T_mat[:3, 3] = np.array([x, y, z])
        return T_mat


    def zd_pose1_callback(self, msg):
        self.zd_pose1 = np.array( [msg.pose.position.x,
                                      msg.pose.position.y,
                                      msg.pose.position.z,
                                      msg.pose.orientation.x,
                                      msg.pose.orientation.y,
                                      msg.pose.orientation.z,
                                      msg.pose.orientation.w])

    def zd_pose2_callback(self, msg):
        self.zd_pose2 = np.array( [msg.pose.position.x,
                                      msg.pose.position.y,
                                      msg.pose.position.z,
                                      msg.pose.orientation.x,
                                      msg.pose.orientation.y,
                                      msg.pose.orientation.z,
                                      msg.pose.orientation.w])


    def control_rot(self):

        for i in range(20):
        # while not rospy.is_shutdown():

            zd_pose1 = self.zd_pose1
            zd_pose2 = self.zd_pose2
            obj1_in_cam1 = self.pose_to_matrix(zd_pose1)
            obj2_in_cam2 = self.pose_to_matrix(zd_pose2)
            cam1_in_cam2 = np.array([[0.99445312, -0.00209779, 0.10518245, 0.0297208],
                                      [0.00369752, 0.9998808, -0.01501967, 0.63169346],
                                      [-0.10513859, 0.01532548, 0.99434156, -0.01338732],
                                      [0., 0., 0., 1.]])
            obj1_in_cam2 = np.dot(cam1_in_cam2,obj1_in_cam1)

            target_obj1_in_cam2 = np.array([[-0.0137449 , -0.99765105 , 0.06714326 , 0.0174374 ],
                                    [ 0.99978385, -0.01266235 , 0.01651668  ,0.68542393],
                                    [-0.01562745 , 0.06735598,  0.99760868 , 0.74580226],
                                    [ 0.        ,  0.       ,   0.        ,  1.        ]])
            target_obj2_in_cam2 = np.array([[-0.09200589, -0.99155748 , 0.0913711,   0.05668435],
                                    [ 0.99564808, -0.09024133 , 0.02326795,  0.04420056],
                                    [-0.01482606,  0.09311425 , 0.99554504,  0.76173723],
                                    [ 0.        ,  0.   ,       0.       ,   1.        ]])

            target_p1 = np.array([target_obj1_in_cam2[0,3], target_obj1_in_cam2[1,3]])
            target_p2 = np.array([target_obj2_in_cam2[0,3], target_obj2_in_cam2[1,3]])
            current_p1 = np.array([obj1_in_cam2[0,3], obj1_in_cam2[1,3]])
            current_p2 = np.array([obj2_in_cam2[0,3], obj2_in_cam2[1,3]])

            # print("target_p1 : {}, target_p2 : {}".format(target_p1,target_p2))
            # print("current_p1 : {}, current_p2 : {}".format(current_p1,current_p2))

            # 计算向量 AB 和 CD
            vec_ab = target_p2 - target_p1
            vec_cd = current_p2 - current_p1
            # 计算点积
            dot_product = np.dot(vec_ab, vec_cd)

            # 计算向量模长
            norm_ab = np.linalg.norm(vec_ab)
            norm_cd = np.linalg.norm(vec_cd)
            # 计算夹角的余弦值
            cos_theta = dot_product / (norm_ab * norm_cd)
            # 计算夹角的绝对值（弧度）
            theta_rad = np.arccos(cos_theta)
            # 计算叉积
            cross_product = np.cross(vec_ab, vec_cd)
            # 根据叉积符号确定夹角方向
            if cross_product < 0:
                theta_rad = -theta_rad
            # 转换为角度
            theta_deg = np.degrees(theta_rad)
            print(f"带符号的夹角为：{theta_deg:.2f} 度")


            left_righ_error1 =target_p1[0] - current_p1[0]
            left_righ_error2 =target_p2[0] - current_p2[0]
            print(left_righ_error1)
            print(left_righ_error2)


            if abs(theta_deg) > 0.1: # 先调整旋转角度
                if theta_deg>0:
                    command = MecanumCommand()
                    command.motion_type = 'turn_ccw'
                    command.accel_time = 0.1
                    command.cruise_time = 0.08 * abs(theta_deg)
                    command.decel_time = 0.1
                    command.target_rpm = 50.0
                    self.cmd_pub.publish(command)
                    time.sleep(0.2 +  0.08 * abs(theta_deg))
                else:
                    command = MecanumCommand()
                    command.motion_type = 'turn_cw'
                    command.accel_time = 0.1
                    command.cruise_time = 0.08  * abs(theta_deg)
                    command.decel_time = 0.1
                    command.target_rpm = 50.0
                    self.cmd_pub.publish(command)
                    time.sleep(0.2 +  0.08 * abs(theta_deg))

            else:
                if left_righ_error1 > 0:
                    command = MecanumCommand()
                    command.motion_type = 'right'
                    command.accel_time = 0.2
                    command.cruise_time = 0.8 * abs(theta_deg)
                    command.decel_time = 0.2
                    command.target_rpm = 80.0
                    self.cmd_pub.publish(command)
                    time.sleep(0.4 +  0.8 * abs(theta_deg))
                else:
                    command = MecanumCommand()
                    command.motion_type = 'left'
                    command.accel_time = 0.2
                    command.cruise_time = 0.8 * abs(theta_deg)
                    command.decel_time = 0.2
                    command.target_rpm = 80.0
                    self.cmd_pub.publish(command)
                    time.sleep(0.4 +  0.8 * abs(theta_deg))


    def spin(self):
        time.sleep(1)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.control_rot()
            break
            rate.sleep()








if __name__ == '__main__':
    car_controler = Car_Controller()
    try:
        car_controler.spin()
    except KeyboardInterrupt:
        print("shutting down")






