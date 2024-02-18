#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import PoseWithCovariance, Twist
# 速度の型
from nav_msgs.msg import Odometry
# 自分の位置情報の格納の型
from sensor_msgs.msg import LaserScan
# LiDERの情報を格納するためのメッセージの型
from tf.transformations import euler_from_quaternion
# モータの角度を知るための座標変換のクラス

import math
# 標準モジュール
import numpy as np

class TestProgramNode:
    def __init__(self):
        # ノードの初期化
        rospy.init_node("TestProgramNode")
        
        # 動作周期を設定(単位はHz)
        self.loop_rate = rospy.Rate(10)

        # オドメトリとURGの情報を受け取るためのSubscriber
        self.odom_sub = rospy.Subscriber("ypspur_ros/odom", Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        #速度、角速度のトピックを送るためのPublisher
        self.cmd_vel_pub = rospy.Publisher("ypspur_ros/cmd_vel", Twist, queue_size=100)

        # 現在のロボットの位置，姿勢を格納する変数
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0 

        # 送る速度と角速度のトピック
        self.twist = Twist()
        self.twist.linear.x = 0.0 # 速度
        self.twist.angular.z = 0.0 # 角速度

        # 受け取ったセンサのトピック
        self.scan = LaserScan()


    # 指定した動作をさせるための関数
    def run(self):
        while not rospy.is_shutdown():
            self.loop_rate.sleep()

            # 処理したい内容をここに記述
            
            self.cmd_vel_pub.publish(self.twist)
            

    # /scanが更新されるたびに呼び出されるコールバック関数
    def scan_callback(self, msg):
        # よく使うトピックの内容
        self.scan.ranges = msg.ranges # センサの取得した距離(配列に格納されている)
        self.scan.angle_min = msg.angle_min # センサの測定できる範囲の角度の最小値
        self.scan.angle_max = msg.angle_max # センサの測定できる範囲の角度の最大値
        self.scan.angle_increment = msg.angle_increment # 何度毎にセンサの値が格納されているか

        # その他に格納されている内容
        self.scan.header = msg.header # タイムスタンプやフレームの情報
        self.scan.time_increment = msg.time_increment # 何秒毎にセンサの値が格納されているか
        self.scan.scan_time = msg.scan_time # 全てのセンサデータを測定するまでにかかった時間
        self.scan.range_min = msg.range_min # センサの測定できる距離の最小値
        self.scan.range_max = msg.range_max # センサの測定できる距離の最大値
        self.scan.intensities = msg.intensities # センサの取得した反射強度(配列に格納されている)

    # /ypspur_ros/odomが更新される度に呼び出されるコールバック関数
    def odom_callback(self, msg):
        # ロボットの(x,y)座標
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # 現在のロボットの姿勢を計算
        self.robot_yaw = self.geometry_quat_to_rpy(msg.pose.pose.orientation)

    # クォータニオンをオイラーに変換
    def geometry_quat_to_rpy(self, geometry_quat):
        (_, _, yaw) = euler_from_quaternion([geometry_quat.x, geometry_quat.y, geometry_quat.z, geometry_quat.w])
        return yaw

if __name__ == "__main__":
    test_program_node = TestProgramNode()
    test_program_node.run()