#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import actionlib
import rospy
from std_msgs.msg import String 
import rospy
import threading
import pygame
from rois_env.msg import *
from rois_env.srv import *
from datetime import datetime
import yaml
import face_recognition
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import psycopg2 as pg
import inspect
import cv2

#サンプルコンポーネント
#int64 time_to_run

# PostgreSQL接続情報

home_path = os.environ['HOME']
db_file_path = home_path + '/database.yml'
with open(db_file_path, 'r') as file:
    db = yaml.safe_load(file)


db_config = {
    'host': 'localhost',
    'dbname': db['dbname'],
    'user': db['user'],
    'password': db['password'],
}



directry = home_path +"/catkin_ws/src/rois_env/script/voice/identification.mp3"
file_path = home_path + '/catkin_ws/src/sensor_sytem/script/person_id.yml'

class Person_IdentificationService:
    def __init__(self):
        #初期化できていないためコンポーネントの状態をUNINITIALIZEDにする
        self.comp_state = "UNINITIALIZED"
        print(self.comp_state)
        #コンポーネント名を設定する
        self.comp_ref = "Person_Identification"
        print(self.comp_ref)

        self.robotname =  '/'+ self.get_robotfile()["Robot"]
        print(f"Robot name: {self.robotname}")

         #コンポーネントの状態確認のサービス設定
        state_name = self.robotname + '/get_state/'+self.comp_ref
        self.comp_state_service = rospy.Service(state_name, component_status, self.component_status)  

        ##identify関係init##
        # CvBridgeのインスタンスを作成
        self.bridge = CvBridge()
        # 必要に応じて画像ファイルを読み込み、エンコードを追加
        # 既知の顔エンコーディングと名前を設定(本当はyamlがいい)
        self.known_face_encodings = []
        self.known_face_names = []
        self.known_face_encodings.append(face_recognition.face_encodings(face_recognition.load_image_file("/home/rsdlab/catkin_ws/src/rois_ros/script/images/manato.jpg"))[0])
        self.known_face_names.append("Manato Fukuta")
        #self.known_face_encodings.append(face_recognition.face_encodings(face_recognition.load_image_file("/home/rsdlab/catkin_ws/src/rois_ros/script/images/karina.jpg"))[0])
        #self.known_face_names.append("Karina")
        self.known_face_encodings.append(face_recognition.face_encodings(face_recognition.load_image_file("/home/rsdlab/catkin_ws/src/rois_ros/script/images/ohara.jpg"))[0])
        self.known_face_names.append("Kenichi Ohara")
        self.known_face_encodings.append(face_recognition.face_encodings(face_recognition.load_image_file("/home/rsdlab/catkin_ws/src/rois_ros/script/images/fukuda.jpg"))[0])
        self.known_face_names.append("Toshio Fukuda")

        ##

        #Command用のアクション通信の設定(!!!)
        exe_name =self.robotname + '/execute/' + self.comp_ref
        self.server = actionlib.SimpleActionServer(exe_name, executeAction, self.execute, False)
        self.server.start()
        rospy.loginfo(f"{exe_name} Service is ready.")


        self.pub = rospy.Publisher(self.robotname + '/event_identified', notify_identified , queue_size=1)

        # self.set = rospy.Service('/test_set_param', test_set_param, self.set_parameter)  
        # self.get = rospy.Service('/test_get_param', test_get_param, self.get_parameter)
        # self.pub = rospy.Publisher('/complete_command', completed , queue_size=1)
        # self.pub = rospy.Publisher('/completed', String,queue_size=1)

        self._set = 10

        # サービスサーバーを作成

        rospy.loginfo("TestComponent Service is ready.")

        #Thread用の設定
        self.state = "idle"
        self.current_goal = None
        #準備できたらコンポーネントの状態をREADYにする
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  #コンポーネントをREADY状態にする

        self.SEND_TEXT = False         #speech_recognizedメソッド用
        self.playback_thread = None 

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
        

    def set_parameter(self, s_req):
        print("set parameter")

        try:
            self._set = s_req.time_to_run
            set_return = "OK"
            return test_set_paramResponse(set_return) 

        except:
            set_return = "BAD_PARAMATER"
            return test_set_paramResponse(set_return)

        print("speech_words: ",self._test)
           
                

    def get_parameter(self,g_req):
        print("get_parameter")
        print(self._test)
        return speech_get_paramResponse(self._test)


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
            print(self.command)
            self.suspend()

        elif self.command == "resume":
            self.resume()

        else:
            rospy.loginfo("No valid command received.")
            self.result.success = "False"
            self.server.set_aborted(self.result)


    def count(self):
        print("count")

    def start(self):
        self.state = "playing"
        print(self._set)

        self.result.success = "True"
        self.server.set_succeeded(self.result)

        # 画像受け取り開始
        self.sub_thread = threading.Thread(target=self.subscribe_image)
        self.sub_thread.start()    
        # スレッドが終了するのを待つ
        self.sub_thread.join()
        print("jjj22")

        # 人認識開始
        print("start")
        self.playback_thread = threading.Thread(target=self.identify)
        self.playback_thread.start()
    
    def subscribe_image(self):
        rospy.loginfo("Subscribing to /torobo/camera/image_raw...")
        #self.image_sub = rospy.Subscriber('/torobo/head/see3cam_left/camera/color/image_raw', Image, self.camera_callback)
        # self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_callback)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)

        # rospy.spin()  # 画像購読を維持するためにspinを実行
        print("jjj")
        rospy.sleep(3)

    def camera_callback(self, msg):
        try:
            # ROSのImageメッセージをOpenCVの画像に変換
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # クロップ範囲の指定（例: x=400, y=400, 幅=500, 高さ=400）
            x, y, w, h = 400, 300, 500, 400
            # 画像のサイズを取得して、クロップ範囲を制限
            height, width = self.cv_image.shape[:2]
            x_end = min(x + w, width)
            y_end = min(y + h, height)
            # 指定範囲をクロップ
            cropped_image = self.cv_image[y:y_end, x:x_end]
            # クロップした画像をRGB形式に変換
            self.cropped_image_rgb = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB)
            rospy.loginfo("Image received from /camera/image_raw.")
            # print(inspect.getouterframes(inspect.currentframe())[1])
            # self.process_image(self.cv_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))


    def stop(self):
        print("stop")
        # if self.state == "playing":
        #     self.state = "stopped"
        #     self.feedback.status = "playing stopped."
        #     rospy.sleep(3)
        #     pub_data = completed()
        #     pub_data.command_id = self.command
        #     pub_data.status = "succeeded"
        #     self.pub.publish(pub_data)
            
        #     self.result.success = "True"
        #     self.server.set_succeeded(self.result)
        # else:
        #     rospy.logwarn("No active playing.")
        #     self.feedback.status = "No active goal to stop."
        #     self.result.success = "False"
        #     self.server.set_aborted(self.result)


    def suspend(self):
        print("suspend")
        # if self.state == "playing":
        #     self.state = "suspended"
        #     self.feedback.status = "play suspended."
        #     self.result.success = "True"
        #     self.server.set_succeeded(self.result)
        # else:
        #     rospy.logwarn("Cannot suspend; not playing.")
        #     self.result.success = "False"
        #     self.server.set_aborted(self.result)


    def resume(self):
        print("resume")
        # if self.state == "suspended":
        #     # pygame.mixer.music.unpause()
        #     self.state = "playing"
        #     self.feedback.status = "Playing resumed."
        #     self.result.success = "True"
        #     self.server.set_succeeded(self.result)

        # else:
        #     rospy.logwarn("No previous goal to resume.")
        #     self.result.success = "False"
        #     self.server.set_aborted(self.result)

    def person_identified(self):
        completed_time= rospy.get_time()
        self.completed_time =str(datetime.fromtimestamp(completed_time))
        print(self.completed_time)

        #通知するトピックの設定
        pub_data = notify_identified()
        pub_data.event_type = "person_identified"
        pub_data.timestamp = self.completed_time
        #認識結果を格納
        pub_data.person_ref = self.person_ref


        self.pub.publish(pub_data)
            


    def component_status(self, req):

        if (req.component_name == self.comp_ref):
            rospy.loginfo("Current state requested: %s", self.comp_state)
            return component_statusResponse(self.comp_state)
        else:
            pass

    def identify(self):
        print("jjj333")
        
        # 名前が"Unknown"でない場合までループ
        name = "Unknown"
        while name == "Unknown":
            # 顔の位置とエンコーディングを検出
            face_locations = face_recognition.face_locations(self.cropped_image_rgb)
            face_encodings = face_recognition.face_encodings(self.cropped_image_rgb, face_locations)
            
            # 検出された顔それぞれに対して認識を行う
            for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
                name = "Unknown"

                face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
                best_match_index = face_distances.argmin()

                if matches[best_match_index]:
                    name = self.known_face_names[best_match_index]

                print(f"Detected name: {name}")

                if name != "Unknown":
                    pygame.mixer.init()
                    pygame.mixer.music.load(directry)
                    pygame.mixer.music.play()
                
                    # データベースからid取得
                    connection = self.connect_to_database()
                    if not connection:
                        return
                    
                    with connection.cursor() as cursor:
                        cursor.execute("""
                            SELECT id from list 
                            WHERE name = %s
                        """, (name,))
                        result = cursor.fetchall()
                        self.person_ref = result[0][0]
                        print(f"Person ID: {result[0][0]}")

                        with open(file_path, 'r') as file:
                            data = yaml.safe_load(file)
                        data['id'] = self.person_ref  # 「id」値を更新

                        # 更新したデータをYAMLファイルに書き込み
                        with open(file_path, 'w') as file:
                            yaml.safe_dump(data, file)

                    break  # 名前が確定したらループを抜ける



            # 認識できなかった場合、再度顔認識処理を行う前に待機する（不要な場合は削除）
            if name == "Unknown":
                rospy.loginfo("Name is still unknown, retrying...")
                rospy.sleep(1)  # 少し待機して再実行

        # 名前が判明した場合、person_identifiedメソッドを呼ぶ
        self.person_identified()
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  #コンポーネントをREADY状態にする

        return None  

    
    def connect_to_database(self):
        try:
            connection = pg.connect(**db_config)
            return connection
        except pg.Error as e:
            print(f"Error: Unable to connect to the database\n{e}")
            return None


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

    node_name = robot + '_Person_Identification'    
    rospy.init_node(node_name)
    service = Person_IdentificationService()
    service.run()
    

