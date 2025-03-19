#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import yaml
import subprocess
import rosparam
from std_msgs.msg import String
import pygame
from rois_env.srv import *
from rois_env.msg import *
import actionlib
import threading
from datetime import datetime

#eventにも挑戦


class Person_LocalizationService():
    def __init__(self,robotname):

        #初期化できていないためコンポーネントの状態をUNINITIALIZEDにする
        self.comp_state = "UNINITIALIZED"
        print(self.comp_state)        
        
        #コンポーネント名を設定する
        self.comp_ref = "Person_Localization"

        self.robotname = robotname
        
        #コンポーネントの状態確認のサービス設定
        state_name = self.robotname + '/get_state/' + self.comp_ref
        self.state_service = rospy.Service(state_name, component_status, self.component_status)



        #Command用のアクション通信の設定
        exe_name = self.robotname + '/execute/' + self.comp_ref
        self.server = actionlib.SimpleActionServer(exe_name, executeAction, self.execute, False)
        self.server.start()
        rospy.loginfo(f"{exe_name} Service is ready.")
        

        # #パラメータ操作用のサービス通信の設定
        self.localized_human = rospy.ServiceProxy('/get_position', GetPosition)
        rospy.wait_for_service('/get_position')

        #Thread用の設定
        self.state = "idle"
        self.localized_thread = None

        self.position_data = [0,0,0]


        #イベント通知するためのトピック通信の設定
        self.pub = rospy.Publisher(self.robotname + '/event_localization', notify_localrec , queue_size=1)

        #コマンド終了の通知用のトピックの設定
        self.pub1 = rospy.Publisher(self.robotname + '/completed_event', completed , queue_size=1)

        #イベント通知のトピックを送信するか判断する変数の設定
        self.SEND_DETECT = False         #person_detectedメソッド用

        #準備できたらコンポーネントの状態をREADYにする
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  #コンポーネントをREADY状態にする

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
        self.comp_state = "BUSY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  #コンポーネントをBUSY状態にする

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

    # def detect_cb(self):
    #     self.JUDGEMENT = 

    def start(self):
        print("start")

        self.state = "playing"

        self.result.success = "True"
        self.server.set_succeeded(self.result)

        self.localized_thread = threading.Thread(target=self.localization)
        self.localized_thread.start()


    def stop(self):
        print("stop")
      
    def suspend(self):
        print("suspend")

    def resume(self):
        print("resume")


    def subs(self, goal):
        self.event = goal.event_type
        
        #イベントメソッド名を指定
        if self.event == "person_localized":
            #トピックを送信するように設定
            self.SEND_TEXT = True
            #返答
            return subscribe_setResponse("True")

        #該当するイベントメソッド名がない場合
        else:
            rospy.loginfo("No valid command received.")
            return subscribe_setResponse("False")



    def localization(self):
        rospy.loginfo("Monitoring playback.")
        if self.state == "playing":

            response = self.localized_human()
             
            print(response.position_data)


            completed_time= rospy.get_time()
            
            self.completed_time =str(datetime.fromtimestamp(completed_time))


            if not response.position_data == [0,0,0]:
                self.person_localized(self.completed_time, response.position_data)
                               
                return



    def person_localized(self,timestamp,position_data):
        
        int_array = []

        for num in position_data:
            int_array.append(int(num))

        pub_data = notify_localrec()
        pub_data.event_type = "person_localized"
        pub_data.timestamp = timestamp
        pub_data.position_data = int_array

        print(pub_data)


        
        self.pub.publish(pub_data)
        
        
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')
       


    #完了したら呼び出される関数
    def on_detection_complete(self,timestamp,position_data):
        rospy.loginfo("localize completed successfully.")

        #認識が完了した時間と認識結果を通知するメソッド
        self.person_localized()

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
    robot =  get_robotfile()["Robot"]
    robotname =  '/'+ robot
    print(f"Robot name: {robotname}")

    node_name = robot + '_Person_Localization'
  
    rospy.init_node(node_name)
    server = Person_LocalizationService(robotname)
    server.run()