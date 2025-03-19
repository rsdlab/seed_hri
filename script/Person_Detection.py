#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import yaml
import subprocess
import rosparam
from std_msgs.msg import String,Bool
import pygame
from rois_env.srv import *
from rois_env.msg import *
import actionlib
import threading
from datetime import datetime



class Person_DetectionService():
    def __init__(self,robotname):

        #初期化できていないためコンポーネントの状態をUNINITIALIZEDにする
        self.comp_state = "UNINITIALIZED"
        print(self.comp_state)        
        
        #コンポーネント名を設定する
        self.comp_ref = "Person_Detection"

        self.robotname =  robotname

        #コンポーネントの状態確認のサービス設定
        state_name = self.robotname + '/get_state/' + self.comp_ref
        self.state_service = rospy.Service(state_name, component_status, self.component_status)



        #Command用のアクション通信の設定
        exe_name = self.robotname + '/execute/' + self.comp_ref
        self.server = actionlib.SimpleActionServer(exe_name, executeAction, self.execute, False)
        self.server.start()
        rospy.loginfo(f"{exe_name} Service is ready.")


        # #パラメータ操作用のサービス通信の設定
        self.detect_human = rospy.ServiceProxy('/detection', detection)  
        rospy.wait_for_service( '/detection')
        self.detect_human1 = rospy.ServiceProxy('/start', JudgeParam)  
        rospy.wait_for_service( '/start')
        self.detect_human2 = rospy.ServiceProxy('/stop', JudgeParam)  
        rospy.wait_for_service( '/stop')

        #Thread用の設定
        self.state = "idle"
        self.detected_thread = None

        self.num = 0


        #イベント通知するためのトピック通信の設定
        self.pub = rospy.Publisher(self.robotname + '/event_persondetec', notify_persondetect , queue_size=1)

        #コマンド終了の通知用のトピックの設定
        self.pub1 = rospy.Publisher('/completed_event', completed , queue_size=1)

        self.detect_sub = rospy.Subscriber('/judge_param', Bool , self.decb)
        #イベント通知のトピックを送信するか判断する変数の設定
        self.SEND_DETECT = False         #person_detectedメソッド用

        #準備できたらコンポーネントの状態をREADYにする
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  


    def get_robotfile(self):
        # 現在のスクリプトの絶対パスを取得
        current_file_path = os.path.abspath(__file__)

        package_relative_path = current_file_path.split('/src/')[1]
        catkin_path = current_file_path.split('/src/')[0]
        package_name = package_relative_path.split('/')[0]

        path = catkin_path +'/src/'+ package_name +"/robot.yaml"
        
        with open(path, 'r') as file:
            data = yaml.safe_load(file)

        return data

    #コンポーネントが持つパラメータを設定するためのset_parameter()
    def set_parameter(self, s_req):
        print("set parameter")



    def get_parameter(self,g_req):
        print("get_parameter")


    def execute(self, goal):
        #コンポーネントをBUSY状態にする
        self.comp_state = "BUSY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  

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



    def start(self):
        print("start")

        self.state = "playing"

        self.result.success = "True"
        self.server.set_succeeded(self.result)

        self.detected_thread = threading.Thread(target=self.detection)
        self.detected_thread.start()


    def stop(self):
        print("stop")




    def suspend(self):
        print("suspend")




    def resume(self):
        print("resume")




    def decb(self,msg):
        print(msg.data)
        if not msg.data:
            print("return")
            return
        else:
            self.num = msg.data
            print(self.num)


    #音声認識
    def detection(self):
        rospy.loginfo("Monitoring playback.")
        if self.state == "playing":

            response = self.detect_human1()
            # response = self.detect_human("start")
             
            print(response)

            while self.num <1:
                print("detect...")
                rospy.sleep(0.1)

            completed_time= rospy.get_time()
            
            self.completed_time =str(datetime.fromtimestamp(completed_time))

            response = self.detect_human2()
            
            self.on_detection_complete(self.completed_time,self.num)
            # if response.result == "success":
                # self.person_detected(response.timestamp,response.number)
                               
                # return



    def person_detected(self,timestamp,number):
        print(timestamp,number)
        
        pub_data = notify_persondetect()
        pub_data.event_type = "person_detected"
        pub_data.timestamp = timestamp
        pub_data.number = 1

        # pub_data = notify_persondetect()
        # pub_data.event_type = "person_detected"
        # pub_data.timestamp = timestamp
        # pub_data.number = number

        print(pub_data)


        self.pub.publish(pub_data)
        
        
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')
       


    #完了したら呼び出される関数
    def on_detection_complete(self,timestamp,number):
        rospy.loginfo("recognize completed successfully.")

        #認識が完了した時間と認識結果を通知するメソッド
        self.person_detected(timestamp,number)

        #READY状態にする
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}') 



    def component_status(self, req):
        # 現在の状態を返答
        if (req.component_name == self.comp_ref):
            rospy.loginfo("Current state requested: %s", self.comp_state)
            return component_statusResponse(self.comp_state)
        else:
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
    robot = get_robotfile()["Robot"]
    robotname =  '/'+ robot
    print(f"Robot name: {robotname}")
    node_name = robot + '_Person_Detection'

    rospy.init_node(node_name)
    service = Person_DetectionService(robotname)
    service.run()


 