#!/bin/env python3
# coding=utf-8
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from rospy.exceptions import ROSInterruptException


class controllerNode:
    def __init__(self):
        #ノードの初期化
        rospy.init_node("controllerNode")
        
        #ノードのSubscriberを設定
        #第一引数:Subscribe対象トピック, 第二引数:受け取るメッセージの型, 第三引数:メッセージを受け取った際のコールバック関数
        #Subscribe対象のトピックが更新される度にメッセージを受け取り、コールバック関数が呼び出される
        self.sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        #ノードのPublisherを設定
        #第一引数:Publish先トピック, 第二引数:Publishするメッセージの型, queue_sizeはメッセージを保持しておく数？だった気がする
        #.publish(hoge)メソッドでhogeをトピックにPublish
        self.pub = rospy.Publisher('joy_state', Float32, queue_size=10)
        
    #Joyが更新されるたび（コントローラ入力がある度に）呼び出されるコールバック関数
    def joy_callback(self, joy_msg):
        #axes[6]:左右キー 1.0で左入力, -1.0で右　axes[7]:上下キー  1.0で上入力, -1.0で下
        #axes[0]:左スティック左右(左が正)　axes[1]:左スティック上下(上が正)
        
        #空のメッセージを作成
        state = Float32()
        
        #コントローラの入力をメッセージに格納
        state.data = joy_msg.axes[1]
        
        #Subscribeしたコントローラの入力を標準出力
        for i in range(4):
            rospy.loginfo(str(i+1), joy_msg.axes[i])
        rospy.loginfo("\n")
        
        #自身のPublisherでメッセージをPublish
        self.pub.publish(state)
        ##ログを出力
        # rospy.loginfo("linear_x: %f, angular_z: %f", twist.linear.x, twist.angular.z)
        
            
if __name__ == '__main__':
    try:
        rospy.loginfo("CONTROLLER START")
        controllerNode()
        rospy.spin()
    except ROSInterruptException:
        rospy.loginfo("CONTROLLER START FAILED")
        pass