#!/usr/bin/env python

import rospy
import os
import yaml
import math
import threading
import tf
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rois_env.srv import *
from rois_env.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveService:
    def __init__(self,robotname):
        #HRI-Cの状態をUNINITIALIZEDにする
        self.comp_state = "UNINITIALIZED"
        rospy.loginfo(f'Componemt status: {self.comp_state}')

        self.comp_ref = "Move"

        self.robotname = robotname
        print(f"Robot name: {self.robotname}")

        state_name = self.robotname + '/get_state/' + self.comp_ref
        self.state_service = rospy.Service(state_name, component_status, self.component_status)

        #アクションサーバーの立ち上げ
        exe_name =self.robotname + '/execute/' + self.comp_ref
        self.server = actionlib.SimpleActionServer(exe_name, executeAction, self.execute, False)
        self.server.start()

        #コンポーネント側のset_parameter,get_parameterのサービスサーバー
        self.set = rospy.Service(self.robotname + '/move_set_param', move_set_param, self.set_parameter) 
        self.get = rospy.Service(self.robotname + '/move_get_param', move_get_param, self.get_parameter) 

        self.status = "idle"
        self.current_goal = None
        
        #動作が完了したときに送信するトピック
        self.comp_pub = rospy.Publisher(self.robotname + '/completed_command', completed , queue_size=1)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        
        
        # 移動の目標相対座標
        self.relative_target_x = 0.30  # x方向の相対移動距離
        self.relative_target_y = 0   # y方向の相対移動距離
        self.relative_target_yaw = 0
        # 初期位置
        self.start_x = None
        self.start_y = None
        self.current_x = 0.0
        self.current_y = 0.0

        # PI制御用の変数
        self.Kp = 0.5  # 比例ゲイン
        self.Ki = 0.1  # 積分ゲイン
        self.error_sum_x = 0.0
        self.error_sum_y = 0.0
        self.last_time = rospy.Time.now().to_sec()
        self.error_x = 0.0
        
        self.target_x = 2200
        self.target_y =  200
        self.target_yaw = 0

        self.target_rad = 0
        self.target_dir = 0

        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')

        rospy.loginfo("Move  is ready.")



    #Engineからのパラメータ取得に応じる
    def get_parameter(self,g_req):
        print("get_parameter")
        _target_x = int(self.relative_target_x * 1000)
        _target_y = int(self.relative_target_y * 1000)
        _target_yaw = int(self.relative_target_yaw)

        _line  = [_target_x, _target_y, _target_yaw]
        _curve = [self.target_rad, self.target_dir]

        print(_line)

        return move_get_paramResponse(_line)


    #Engineからのパラメータ設定に応じる
    def set_parameter(self,s_req):
        print("set parameter")

        try:
            self.relative_target_x   = s_req.line[0] / 1000
            self.relative_target_y   = s_req.line[1] / 1000
            self.relative_target_yaw = s_req.line[2] / 1000
            # self.set_rad = s_req.curve[0]
            # self.set_c_deg = s_req.curve[1]
            rospy.loginfo(self.relative_target_x)
            rospy.loginfo(self.relative_target_y)
            rospy.loginfo(self.relative_target_yaw)


            value = "OK"
            return move_set_paramResponse(value)
        except:
            value = "BAD PARAMETER"
            print(value)
            return move_set_paramResponse(value)  


    def odom_callback(self, msg):
        """現在のx, y座標を取得"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # 初期位置の設定
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            rospy.loginfo(f"Start position: ({self.start_x}, {self.start_y})")


    #コマンド実行の関数
    def execute(self, goal):
        #コンポーネントをBUSY状態にする
        self.comp_state = "BUSY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  

        if self.start_x is None or self.start_y is None:
            rospy.logwarn("Waiting for initial position...")
            while self.start_x is None or self.start_y is None:
                rospy.sleep(0.1)  # 少し待機
        #コマンド
        self.command = goal.command_name
        rospy.loginfo("Received command: %s", self.command)

        self.feedback = executeFeedback()
        self.result = executeResult()


        if self.command == "start":
            self.start()
            
        elif self.command == "stop":
            self.stop()

        elif self.command == "suspend":
            self.suspend()

        elif self.command == "resume":
            self.resume()

        else:
            rospy.loginfo("No valid command received.")
            self.result.success = "False"
            self.server.set_aborted(self.result)

        #コンポーネントをREADY状態にする
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  

    def move_robot(self):
        # ロボットを動かす処理
        rate = rospy.Rate(10)  # 10Hzでループ
        velocity = Twist()
        print(f"x: {self.current_x},  y: {self.current_y}")
        print(f"x: {self.start_x},  y: {self.start_y}")

        self.status = "moving"
        rospy.loginfo(self.status)

        while not rospy.is_shutdown():
            # 移動距離の誤差を計算
            error_x = (self.start_x + self.relative_target_x) - self.current_x
            error_y = (self.start_y + self.relative_target_y) - self.current_y
            # print(f"x: {self.current_x},")
 

            # 時間差を取得
            current_time = rospy.Time.now().to_sec()
            dt = current_time - self.last_time

            # 積分誤差の更新
            self.error_sum_x += error_x * dt
            self.error_sum_y += error_y * dt

            # PI制御で速度を計算
            linear_x = self.Kp * error_x + self.Ki * self.error_sum_x
            linear_y = self.Kp * error_y + self.Ki * self.error_sum_y

            # 安全のため速度制限（-0.5 ~ 0.5 m/s）
            linear_x = max(min(linear_x, 0.3), -0.3)
            linear_y = max(min(linear_y, 0.3), -0.3)

            # 目標位置を超えないように制御
            if self.current_x + linear_x * 0.1 > (self.start_x + self.relative_target_x):
                linear_x = 0  # 目標を超えたら速度を0にする

            # 速度を設定
            velocity.linear.x = linear_x
            velocity.linear.y = linear_y
            self.pub.publish(velocity)

            # 誤差が小さければ停止
            if abs(error_x) < 0.01 and abs(error_y) < 0.01:
                rospy.loginfo(f"x: {self.current_x}, y: {self.current_y}")
                self.start_x = self.current_x
                self.start_y = self.current_y
                rospy.loginfo("Reached target position.")
                self.status = "completed"
                break
            
            if self.status == "timeup" :
                self.error_x = abs(error_x)
                
                self.start_x = self.current_x
                self.start_y = self.current_y
                break

            # 時間を更新
            self.last_time = current_time
            rate.sleep()

        # 最後に停止
        
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        self.pub.publish(velocity)
        self.completed_command()

    def completed_command(self):
        pub_data = completed()
        pub_data.command_id = "Move"
        pub_data.status = self.status
        print(f"{pub_data.command_id}:{pub_data.status}")
        self.comp_pub.publish(pub_data)



    def limit_time(self):
        num = 8
        while self.status == "moving":
            rospy.sleep(0.1)
    
        for _ in range(num,0,-1):
            print(_)
            rospy.sleep(1)
            if self.status == "completed":
                break
        if not self.status == "completed":
            print("time up")
            self.status = "timeup"
        print(self.status)



    def start(self):        #ナビゲーションのゴールを設定

        self.state = "moving"
        time_thread = threading.Thread(target=self.limit_time)
        time_thread.start()
        move_thread = threading.Thread(target=self.move_robot)
        move_thread.start()

        #コマンドを実行できたと返答する
        self.result.success = "True"
        self.server.set_succeeded(self.result)


    def stop(self):
        if self.state=="moving":
            print("stop")
            velocity = Twist()
            velocity.linear.x = 0.0
            velocity.linear.y = 0.0
            # # velocity.angular.z = 0.0
            # self.pub.publish(velocity)
            self.result.success = "True"
            self.server.set_succeeded(self.result)

        else:
            rospy.logwarn("Cannot suspend; not moving.")
            self.result.success = "False"
            self.server.set_aborted(self.result)

    def suspend(self):
        
        if self.state == "moving":
            #ナビゲーション中なら起動
            if self.move_base_client.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PREEMPTING]:
                #ゴールをキャンセル
                self.move_base_client.cancel_goal()
                self.state = "suspended"

                #コマンドを実行できたと返答する
                self.result.success = "True"
                self.server.set_succeeded(self.result)
            else:
                rospy.logwarn("Cannot suspend; not moving.")
                self.result.success = "False"
                self.server.set_aborted(self.result)
        else:
            rospy.logwarn("Cannot suspend; not moving.")
            self.result.success = "False"
            self.server.set_aborted(self.result)


    def resume(self):
        #suspend状態の場合のみ起動
        if self.state == "suspended":
            if self.current_goal:
                self.move_base_client.send_goal(self.current_goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)
                self.state = "moving"

                self.result.success = "True"
                self.server.set_succeeded(self.result)
            else:
                rospy.logwarn("No previous goal to resume.")
                self.result.success = "False"
                self.server.set_aborted(self.result)
        else:
            rospy.logwarn("Cannot resume; not suspended.")
            self.result.success = "False"
            self.server.set_aborted(self.result)

    def feedback_callback(self, feedback):
        # フィードバックを受け取ったときの処理
        rospy.loginfo("Received feedback: ")



    def component_status(self, req):
        # 現在の状態を返答
        if (req.component_name == self.comp_ref):
            rospy.loginfo("Current state requested: %s", self.comp_state)
            return component_statusResponse(self.comp_state)
        else:
            print("component status pass")
            pass

    def run(self):
        rospy.loginfo("Service node is running...")
        rospy.spin()



def get_robotfile():
    # 現在のスクリプトの絶対パスを取得
    current_file_path = os.path.abspath(__file__)

    package_relative_path = current_file_path.split('/src/')[1]
    catkin_path = current_file_path.split('/src/')[0]
    package_name = package_relative_path.split('/')[0]

    path = catkin_path +'/src/'+ package_name +"/robot.yaml"
    
    with open(path, 'r') as file:
        data = yaml.safe_load(file)

    return data


if __name__ == "__main__":
    robot =  get_robotfile()["Robot"]
    robotname =  '/'+ robot
    print(f"Robot name: {robotname}")

    node_name = robot + '_Move'
  
    rospy.init_node(node_name)
    server = MoveService(robotname)
    server.run()