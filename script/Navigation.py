#!/usr/bin/env python

import rospy
import os
import yaml
import actionlib
from std_msgs.msg import String,Int32
from rois_env.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rois_env.msg import *



class executeServer:
    def __init__(self):
        #HRI-Cの状態をUNINITIALIZEDにする
        self.comp_state = "UNINITIALIZED"
        rospy.loginfo(f'Componemt status: {self.comp_state}')

        self.comp_ref = "Navigation"

        self.robotname =  '/'+ self.get_robotfile()["Robot"]
        print(f"Robot name: {self.robotname}")

        #Component_status用のサービスサーバーの立ち上げ
        state_name = self.robotname + '/get_state/' + self.comp_ref
        self.state_service = rospy.Service(state_name, component_status, self.handle_get_state)  
        rospy.loginfo("component status server is ready")  

        #アクションサーバーの立ち上げ
        exe_name = self.robotname + '/execute/' + self.comp_ref
        self.server = actionlib.SimpleActionServer(exe_name, executeAction, self.execute, False)
        self.server.start()


        #コンポーネント側のset_parameter,get_parameterのサービスサーバー
        self.set = rospy.Service(self.robotname + '/navi_set_param', navi_set_param, self.set_parameter) 
        self.get = rospy.Service(self.robotname + '/navi_get_param', navi_get_param, self.get_parameter) 

        self.state = "idle"
        self.current_goal = None
        
        #動作が完了したときに送信するトピック
        self.pub = rospy.Publisher(self.robotname + '/completed_command', completed , queue_size=1)
        

        
        self.position = [0.0,0.0,0.0,0.0,0.0,0.0,1.0]
        self.select_num = 1
        self.goal_pose = MoveBaseGoal()

      
        # move_baseアクション用クライアント
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()

        
        #HRI-Cの状態をREADYにする
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  #コンポーネントをREADY状態にする

        rospy.loginfo("Navigation Control Action Server is ready.")


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


    #Engineからのパラメータ取得に応じる
    def get_parameter(self,g_req):
        print("get_parameter")
        rospy.loginfo(self.position)
        
        value = self.position
        print(value)
        return navi_get_paramResponse(value)

    #Engineからのパラメータ設定に応じる
    def set_parameter(self,s_req):
        print("set parameter")
        try:
            self.position = s_req.target_position
            rospy.loginfo(self.position)

            #waypointの設定
            self.goal_pose = MoveBaseGoal()
            self.goal_pose.target_pose.header.frame_id = "map"
            self.goal_pose.target_pose.pose.position.x = self.position[0]
            self.goal_pose.target_pose.pose.position.y = self.position[1]
            self.goal_pose.target_pose.pose.position.z = self.position[2]
            self.goal_pose.target_pose.pose.orientation.x = self.position[3]
            self.goal_pose.target_pose.pose.orientation.y = self.position[4]
            self.goal_pose.target_pose.pose.orientation.z = self.position[5]
            self.goal_pose.target_pose.pose.orientation.w = self.position[6]
            #設定できたらOKを返す
            value = "OK" 
            print(value)

            return navi_set_paramResponse(value)
        except Exception as e:
            print(e)
            value = "BAD PARAMETER"
            print(value)
            return navi_set_paramResponse(value)  


    #コマンド実行の関数
    def execute(self, goal):
        #コンポーネントをBUSY状態にする
        self.comp_state = "BUSY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  

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



    def start(self):
        self.current_goal = self.goal_pose
        #ナビゲーションのゴールを設定
        self.move_base_client.send_goal(self.goal_pose, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

        self.state = "navigating"

        #コマンドを実行できたと返答する
        self.result.success = "True"
        self.server.set_succeeded(self.result)


    def stop(self):
        if self.move_base_client.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PREEMPTING]:
            #ゴールをキャンセル
            self.move_base_client.cancel_goal()
            self.state = "stopped"

            #コマンドを実行できたと返答する
            self.result.success = "True"
            self.server.set_succeeded(self.result)
        else:
            rospy.logwarn("No active goal to stop.")
            self.feedback.status = "No active goal to stop."
            self.server.publish_feedback(self.feedback)
            self.result.success = "False"
            self.server.set_aborted(self.result)

    def suspend(self):
        
        if self.state == "navigating":
            #ナビゲーション中なら起動
            if self.move_base_client.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PREEMPTING]:
                #ゴールをキャンセル
                self.move_base_client.cancel_goal()
                self.state = "suspended"

                #コマンドを実行できたと返答する
                self.result.success = "True"
                self.server.set_succeeded(self.result)
            else:
                rospy.logwarn("Cannot suspend; not navigating.")
                self.result.success = "False"
                self.server.set_aborted(self.result)
        else:
            rospy.logwarn("Cannot suspend; not navigating.")
            self.result.success = "False"
            self.server.set_aborted(self.result)


    def resume(self):
        #suspend状態の場合のみ起動
        if self.state == "suspended":
            if self.current_goal:
                self.move_base_client.send_goal(self.current_goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)
                self.state = "navigating"

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

    def done_callback(self, status, result):
        # 結果を受け取ったときの処理
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation goal succeeded.")
            # self.result.success = "True"
            # self.server.set_succeeded(self.result)
            pub_data = completed()
            pub_data.command_id = "navigation"
            pub_data.status = "succeed"
            self.pub.publish(pub_data)

        elif status == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("Navigation goal aborted.")
            # self.server.set_aborted(self.result)
            pub_data = completed()
            pub_data.command_id = "navigation"
            pub_data.status = "aborted"
            self.pub.publish(pub_data)
            
        elif status == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("Navigation goal preempted.")
            # self.server.set_preempted(self.result)

            pub_data = completed()
            pub_data.command_id = "navigation"
            pub_data.status = "preempted"
            self.pub.publish(pub_data)     
        else:
            rospy.logwarn("Navigation goal failed with status: %d", status)
            self.server.set_aborted(self.result)

    def handle_get_state(self, req):
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



if __name__ == '__main__':
    rospy.init_node('navigation_action_server')
    server = executeServer()
    server.run()