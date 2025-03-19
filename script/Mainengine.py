#!/usr/bin/env python3
import rospy
import os
import tf
import yaml
import subprocess
from rois_env.srv import *
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from rois_env.msg import *
from std_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import xml.etree.ElementTree as ET



class EngineService:
    def __init__(self,robotname):
        
        #EngineとApp.が通信できるかどうかを決める変数
        self.ENGINE_RECEIVABLE = False

        self.robotname = robotname

        
        # サービスサーバーを作成
        self.con = rospy.Service('/connect', system_interface, self.connect)
        self.discon = rospy.Service('/disconnect', system_interface, self.disconnect)
        self.bind = rospy.Service('/bind_any', bind_any, self.bind_any)
        self.release = rospy.Service('/release', release, self.release)
        self.service = rospy.Service('/execute', execute, self.execute_cb)
        self.subservice = rospy.Service('/subscribe', subscribe_, self.subscribe_cb)
        self.unsubservice = rospy.Service( '/unsubscribe', unsubscribe_, self.unsubscribe_cb)

        self.eventdetailservice1 = rospy.Service('/recognized_event_detail', get_event_detail_speech_recognized, self.event_recognized_cb)
        self.eventdetailservice2 = rospy.Service('/detected_event_detail', get_event_detail_person_detected, self.event_detected_cb)
        self.eventdetailservice3 = rospy.Service('/local_event_detail', get_event_detail_person_localized, self.event_local_cb)
        self.eventdetailservice4 = rospy.Service('/identified_event_detail', get_event_detail_person_identified, self.event_identified_cb)


        self.completed_sub = rospy.Subscriber( self.robotname + '/completed_command', completed, self.completed)
        self.completed_pub = rospy.Publisher('/completed', completed, queue_size = 1)
        
        self.get_command_result = rospy.Service('/get_command_result', get_command_result, self.get_command_result_cb)
        
        self.command_list = []
        self.event_list = []

        self.speech_recognition_list = []
        self.person_detected_list = []
        self.person_identi_list = []
        self.person_position_list = []

        self.last_timestamp = ""
        
        self.event_count = 0
        self.command_count = 0
        self.subcount = 0

        #　コンポーネントをbindしているかの変数
        self.BINDCOMP = None

        rospy.loginfo("Engine Service is ready.")

        self.bindspeech = 0
        self.bindrecognition = 0


    
    def get_component(self):
        current_file_path = os.path.abspath(__file__)

        package_relative_path = current_file_path.split('/src/')[1]
        catkin_path = current_file_path.split('/src/')[0]
        package_name = package_relative_path.split('/')[0]

        file_path = catkin_path +'/src/'+ package_name +"/hri.xml"

        # XMLファイルをパース
        tree = ET.parse(file_path)
        root = tree.getroot()

        # 名前空間を指定
        namespaces = {
            'rois': 'http://example.com/r/rois',
            'gml': 'http://example.com/r/gml'
        }

        list_ = []
        for hricomponents in root.findall("rois:HRIComponent", namespaces):
            component = hricomponents.text
            name = component.replace("rois:HRIComponent:", "")
            list_.append(name)

        return list_


    #System_Interfaceのconnect()
    #ENGINE_RECEIVABLEがTrueになるとメッセージのやり取りが可能になる
    def connect(self, si_req):
        if si_req.connect == "connect":

            self.ENGINE_RECEIVABLE = True  
            # rospy.loginfo(self.ENGINE_RECEIVABLE)
            rospy.loginfo("Engine is receivable")
            #Returncode_t
            return system_interfaceResponse("OK")           
        else:
            #Returncode_t
            return system_interfaceResponse("ERROR")


    #System_Interfaceの, disconnect()
    #ENGINE_RECEIVABLEがFalseになるとメッセージのやり取りが不可能になる
    def disconnect(self, si_req):
        print("system_interface")
        if si_req.connect == "disconnect":
            self.ENGINE_RECEIVABLE = False
            # rospy.loginfo(self.ENGINE_RECEIVABLE)
            rospy.loginfo("Engine is not receivable")
            #Returncode_t
            return system_interfaceResponse("OK")
      
        else:
            #Returncode_t
            return system_interfaceResponse("ERROR")

    def is_service_available(self,service_name):
        """ROSサービスが存在するか確認"""
        service_list = subprocess.getoutput("rosservice list").split('\n')
        return service_name in service_list

    def is_action_server_active(self,action_name):
        """アクションサーバーが存在しているか確認"""
        try:
            state_topic = f"{action_name}/status"
            rospy.wait_for_message(state_topic, GoalStatusArray, timeout=1.0)
            rospy.loginfo(f"Action server '{action_name}' is active.")
            return True
        except rospy.ROSException:
            rospy.logwarn(f"Action server '{action_name}' is not available.")
            return False    

    #Command_Interfaceのbind_any()
    #component_list_unit内のHRI-Cと接続する関数
    def bind_any(self,b_req):
        
        #エンジンにつながっているHRI-Cの一覧
        # self.component_ref_list = self.get_robotfile()["Component"]
        self.component_ref_list = self.get_component()
        print(self.component_ref_list)

        #ENGINE_RECEIVABLEがTrueの場合
        if self.ENGINE_RECEIVABLE:
            #　self.component_ref…扱うコンポーネントの名前
            self.component_ref = b_req.component_ref
            
            if self.BINDCOMP == None:
            #重複防止のためにサービス名を作成
                get_name =  '/get_parameter/' + self.component_ref  
                set_name =  '/set_parameter/' + self.component_ref
                exe_name = self.robotname + '/execute/' + self.component_ref

                try:
                    #リクエストされたコンポーネント名がcomponent_ref_listにある場合に実行
                    if self.component_ref in self.component_ref_list:
                        #Speech_Synthesisをbindする場合は
                        if self.component_ref == "Speech_Synthesis":
                            print(f"bind Speech_Synthesis time is {self.bindspeech}")
                            if self.bindspeech > 0: 
                                pass
                            else:
                                self.text = "hello"
                                if self.is_service_available(get_name):
                                    rospy.loginfo(f"Service '{get_name}' already exists. Skipping...")
                                else:
                                    self.get_parameter = rospy.Service(get_name, speech_get_parameter, self.get_speech)
                                    rospy.loginfo(f"{get_name} Service is ready.")

                                if self.is_service_available(set_name):
                                    rospy.loginfo(f"Service '{set_name}' already exists. Skipping...")
                                else:
                                    self.set_parameter = rospy.Service(set_name, speech_set_parameter, self.set_speech) 
                                    rospy.loginfo(f"{set_name} Service is ready.")

                                

                            self.client = actionlib.SimpleActionClient(exe_name, executeAction)
                            self.client.wait_for_server()
                            rospy.loginfo(f'action client {exe_name} start...')

                            self.BINDCOMP = self.component_ref 
                            rospy.loginfo(f"Bind {self.component_ref}")
                            self.bindspeech += 1

                            #Returncode_t
                            return bind_anyResponse("OK")


                        elif self.component_ref == "Speech_Recognition":
                            print(f"bind Speech_Recognition time is {self.bindrecognition}")
                            if self.bindrecognition > 0: 
                                pass
                            else:
                                self.languages = "japanese"
                                if self.is_service_available(get_name):
                                    rospy.loginfo(f"Service '{get_name}' already exists. Skipping...")
                                else:
                                    self.get_parameter = rospy.Service(get_name,  s_recognition_get_parameter, self.get_s_recog)
                                    rospy.loginfo(f"{get_name} Service is ready.")
                                
                                if self.is_service_available(get_name):
                                    rospy.loginfo(f"Service '{get_name}' already exists. Skipping...")
                                else:
                                    self.set_parameter = rospy.Service(set_name, s_recognition_get_parameter, self.set_s_recog)
                                    rospy.loginfo(f"{set_name} Service is ready.")

                            self.client = actionlib.SimpleActionClient(exe_name, executeAction)
                            rospy.loginfo(f'action client {exe_name} start...')
                            self.client.wait_for_server() 
                        
                            #  bindしたコンポーネントを格納
                            self.BINDCOMP = self.component_ref 
                            rospy.loginfo(f"Bind {self.component_ref}")
                            self.bindrecognition += 1

                            #Returncode_t
                            return bind_anyResponse("OK")


                        elif self.component_ref == "Move":
                            self.BINDCOMP = self.component_ref 
                            self.line = [500,0,0]
                            if self.is_service_available(get_name):
                                rospy.loginfo(f"Service '{get_name}' already exists. Skipping...")
                            else:
                                self.get_parameter = rospy.Service(get_name,  move_get_parameter, self.get_move)
                                rospy.loginfo(f"{get_name} Service is ready.")
                            if self.is_service_available(set_name):
                                rospy.loginfo(f"Service '{set_name}' already exists. Skipping...")
                            else:
                                self.set_parameter = rospy.Service(set_name, move_set_parameter, self.set_move)
                                rospy.loginfo(f"{set_name} Service is ready.")

                            self.client = actionlib.SimpleActionClient(exe_name, executeAction)
                            rospy.loginfo(f'action server {exe_name} start...')
                            self.client.wait_for_server() 
                            
                            #  bindしたコンポーネントを格納
                            self.BINDCOMP = self.component_ref 
                            rospy.loginfo(f"Bind {self.component_ref}")
                            
                            #Returncode_t
                            return bind_anyResponse("OK")


                        elif self.component_ref == "Person_Detection":
                            self.BINDCOMP = self.component_ref 
                            self.client = actionlib.SimpleActionClient(exe_name, executeAction)
                            rospy.loginfo(f'action server {exe_name} start...')
                            self.client.wait_for_server() 
                            
                            #  bindしたコンポーネントを格納
                            self.BINDCOMP = self.component_ref 
                            rospy.loginfo(f"Bind {self.component_ref}")
                            
                            #Returncode_t
                            return bind_anyResponse("OK")


                        elif self.component_ref == "Person_Localization":
                            self.BINDCOMP = self.component_ref 
                            self.client = actionlib.SimpleActionClient(exe_name, executeAction)
                            rospy.loginfo(f'action server {exe_name} start...')
                            self.client.wait_for_server() 
                            
                            #  bindしたコンポーネントを格納
                            self.BINDCOMP = self.component_ref 
                            rospy.loginfo(f"Bind {self.component_ref}")
                            
                            #Returncode_t
                            return bind_anyResponse("OK")


                        elif self.component_ref == "Person_Identification":
                            self.BINDCOMP = self.component_ref 

                            self.client = actionlib.SimpleActionClient(exe_name, executeAction)
                            rospy.loginfo(f'action server {exe_name} start...')
                            self.client.wait_for_server() 
                            
                            
                            #  bindしたコンポーネントを格納
                            self.BINDCOMP = self.component_ref 
                            rospy.loginfo(f"Bind {self.component_ref}")
                            
                            #Returncode_t
                            return bind_anyResponse("OK")


                        elif self.component_ref == "Approach":
                            self.client = actionlib.SimpleActionClient(exe_name, executeAction)
                            rospy.loginfo(f'action server {exe_name} start...')
                            self.client.wait_for_server() 
                            
                            
                            #  bindしたコンポーネントを格納
                            self.BINDCOMP = self.component_ref 
                            rospy.loginfo(f"Bind {self.component_ref}")
                            
                            #Returncode_t
                            return bind_anyResponse("OK")


                        elif self.component_ref == "Touch":
                            self.client = actionlib.SimpleActionClient(exe_name, executeAction)
                            rospy.loginfo(f'action server {exe_name} start...')
                            self.client.wait_for_server() 
                            
                            
                            #  bindしたコンポーネントを格納
                            self.BINDCOMP = self.component_ref 
                            rospy.loginfo(f"Bind {self.component_ref}")
                            
                            #Returncode_t
                            return bind_anyResponse("OK")



                        elif self.component_ref == "Touch_Detection":
                            self.client = actionlib.SimpleActionClient(exe_name, executeAction)
                            rospy.loginfo(f'action server {exe_name} start...')
                            self.client.wait_for_server() 
                            
                            
                            #  bindしたコンポーネントを格納
                            self.BINDCOMP = self.component_ref 
                            rospy.loginfo(f"Bind {self.component_ref}")
                            
                            #Returncode_t
                            return bind_anyResponse("OK")



                        elif self.component_ref == "Leave":
                            self.client = actionlib.SimpleActionClient(exe_name, executeAction)
                            rospy.loginfo(f'action server {exe_name} start...')
                            self.client.wait_for_server() 
                            
                            
                            #  bindしたコンポーネントを格納
                            self.BINDCOMP = self.component_ref 
                            rospy.loginfo(f"Bind {self.component_ref}")
                            
                            #Returncode_t
                            return bind_anyResponse("OK")

                        elif self.component_ref == "Navigation":
                            self.BINDCOMP = self.component_ref 
                            self.target_position = [0.0,0.0,0.0,0.0,0.0,0.0,1.0]
                            if self.is_service_available(get_name):
                                rospy.loginfo(f"Service '{get_name}' already exists. Skipping...")
                            else:
                                self.get_parameter = rospy.Service(get_name,  navi_get_parameter, self.get_navi)
                                rospy.loginfo(f"{get_name} Service is ready.")
                            if self.is_service_available(set_name):
                                rospy.loginfo(f"Service '{set_name}' already exists. Skipping...")
                            else:
                                self.set_parameter = rospy.Service(set_name, navi_set_parameter, self.set_navi)
                                rospy.loginfo(f"{set_name} Service is ready.")

                            self.client = actionlib.SimpleActionClient(exe_name, executeAction)
                            rospy.loginfo(f'action server {exe_name} start...')
                            self.client.wait_for_server() 
                            
                            #  bindしたコンポーネントを格納
                            self.BINDCOMP = self.component_ref 
                            rospy.loginfo(f"Bind {self.component_ref}")
                            
                            #Returncode_t
                            return bind_anyResponse("OK")
                            


                    #　component_ref_listにない場合
                    else:
                        rospy.loginfo("HRI-Component doesn't exist.")
                        #Returncode_t
                        return bind_anyResponse("ERROR")

                except Exception as e:
                    rospy.logerr(f"An error occurred: {e}")
                    return bind_anyResponse("ERROR")
            else:
                print("Bind_any is failed")
                #Returncode_t
                return bind_anyResponse("ERROR")   

        #ENGINE_RECEIVABLEがFalseの場合
        else:
            print("bind_any is failed")
            #Returncode_t
            return bind_anyResponse("ERROR")   


    #Command_Interfaceのrelease()
    #BindしていたHRI-Cを開放する関数
    def release(self,re_req):
        print("Release")
        try:
            self.get_parameter = None
            self.set_parameter = None 
            self.client = None

            self.BINDCOMP = None
            return releaseResponse("OK")
        except:
            return releaseResponse("ERROR")





    # Command_Interfaceのset_parameter()2
    # Speech_Synthesis用    
    def set_speech(self,s_req):
        print(self.text)

        if self.ENGINE_RECEIVABLE:
            command_id = "set_speech_synthesis"
            self.text = s_req.speech_text
        
            try:
                self.set_text = rospy.ServiceProxy( self.robotname + '/speech_set_param', speech_set_param)
                s_result = self.set_text(self.text)
                print(s_result)

                if s_result.set_return == "OK":
                    return speech_set_parameterResponse(s_result.set_return,command_id)
                else :
                    return speech_set_parameterResponse(s_result.set_return,command_id)

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed")
                returncode_t = "ERROR"
                
                return speech_set_parameterResponse(returncode_t,command_id)
        else:
            print(f"Command {command_id} is failed")
            returncode_t = "ERROR"
            return speech_set_parameterResponse(returncode_t, command_id)

    # Command_Interfaceのset_parameter()
    # Speech_Synthesis用   
    def get_speech(self,g_req):
        print("get_parameter")
        if self.ENGINE_RECEIVABLE:
            try:
                self.get_text = rospy.ServiceProxy(self.robotname + '/speech_get_param', speech_get_param)
                                
                g_text = self.get_text()
                self.text = g_text.speech_text
                print(self.text)
                re = "OK"
                return speech_get_parameterResponse(re,self.text)

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                self.text = " "
                re ="ERROR"
                return speech_get_parameterResponse(re,self.text)
        else:
            print("get parameter is failed")
            self.text = " "
            re ="ERROR"
            return speech_get_parameterResponse(re,self.text)

    def set_s_recog(self,s_req):
        print(self.text)

        if self.ENGINE_RECEIVABLE:
            command_id = "set_speech_recognition"
            self.language = s_req.languages
        
            try:
                self.set_language = rospy.ServiceProxy(self.robotname + '/s_recognition_set_param', s_recognition_set_param)
                s_result = self.set_language(self.text)
                print(s_result)

                if s_result.set_return == "OK":
                    return s_recognition_set_parameterResponse(s_result.set_return,command_id)
                else :
                    return s_recognition_set_parameterResponse(s_result.set_return,command_id)

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed")
                returncode_t = "ERROR"
                
                return s_recognition_set_parameterResponse(returncode_t,command_id)
        else:
            print(f"Command {command_id} is failed")
            returncode_t = "ERROR"
            return s_recognition_set_parameterResponse(returncode_t, command_id)
       
    # Command_Interfaceのget_parameter()
    def get_s_recog(self,g_req):
        print("get_parameter")

        if self.ENGINE_RECEIVABLE:
            print(f"Get parameter {self.BINDCOMP}")
            try: 
                # Navigation Componentからパラメータを取得するサービスのリクエスト
                self.get_language = rospy.ServiceProxy(self.robotname +'/s_recognition_get_param', s_recognition_get_param)
                g_language = self.get_language()

                #目標値を格納
                self.language = g_language.languages
                self.recog_language = g_language.recognizable_languages
                print(self.language)
                print(self.recog_language)

                #できたらOKを返す
                re = "OK"
                return s_recognition_get_parameterResponse(re,self.recog_language, self.language)

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

                #取得できなかったら空の配列とERRORを返す
                self.recog_language =["japanese"]
                self.language = "japanese"
                re ="ERROR"
                return s_recognition_get_parameterResponse(re,self.recog_language, self.language)
        else:
            print("get parameter is failed")
            self.text = " "
            re ="ERROR"
            return  s_recognition_get_parameterResponse(re,self.recog_language, self.language)

    # Command_Interfaceのset_parameter()
    def set_move(self,s_req):

        if self.ENGINE_RECEIVABLE:
            command_id = "set_move"
            self.line  = s_req.line
            self.curve  = s_req.curve
            print(self.line)
        
            try:
                self.set_distance = rospy.ServiceProxy(self.robotname +'/move_set_param', move_set_param)
                s_result = self.set_distance(self.line, self.curve)
                print(1)

                print(s_result)

                if s_result.set_return == "OK":
                    return move_set_parameterResponse(s_result.set_return,command_id)
                else :
                    return move_set_parameterResponse(s_result.set_return,command_id)

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed")
                returncode_t = "ERROR"
                
                return move_set_parameterResponse(returncode_t,command_id)
        else:
            print(f"Command {command_id} is failed")
            returncode_t = "ERROR"
            return move_set_parameterResponse(returncode_t, command_id)



    # Command_Interfaceのset_parameter()

    def get_move(self,g_req):
        print("get_parameter")
        if self.ENGINE_RECEIVABLE:
            try:
                self.get_distance = rospy.ServiceProxy(self.robotname +'/move_get_param', move_get_param)
                g_distance = self.get_distance()
                self.line  = g_distance.line
                print(self.line )
                re = "OK"
                return move_get_parameterResponse(self.line)
                # return move_get_parameterResponse(re,self.line)

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                self.line  = [0,0,0]
                re ="ERROR"
                return move_get_parameterResponse(self.line)
                # return move_get_parameterResponse(re,self.line)
        else:
            print("get parameter is failed")
            self.line  = [0,0,0]
            re ="ERROR"
            return move_get_parameterResponse(self.line)
            # return move_get_parameterResponse(re,self.line)

    # Navigation用    
    def set_navi(self,s_req):

        if self.ENGINE_RECEIVABLE:
            command_id = "set_navigation"
            #　Appからの目標値を格納！
            self.position = s_req.target_position
            try:
                #Navigation Componentにパラメータを設定するサービスのリクエスト
                #引数はtarget_position
                self.set_position = rospy.ServiceProxy(self.robotname +'/navi_set_param', navi_set_param)
                s_result = self.set_position(self.position)
                
                #HRI−Cからの返答
                print(s_result)
                
                #パラメータ設定が成功したかをApp.に返答
                if s_result.set_return == "OK":
                    return navi_set_parameterResponse(s_result.set_return,command_id)
                else :
                    return navi_set_parameterResponse(s_result.set_return,command_id)

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed {e}")
                returncode_t = "ERROR"

                return navi_set_parameterResponse(returncode_t,command_id)
        else:
            print(f"Command {command_id} is failed")
            returncode_t = "ERROR"
            return navi_set_parameterResponse(returncode_t, command_id)

    # Command_Interfaceのget_parameter()
    # Navigation用   
    def get_navi(self,g_req):
        print("get_parameter")

        if self.ENGINE_RECEIVABLE:
            print(f"Get parameter {self.BINDCOMP}")
            # Navigationの場合
            try: 
                # Navigation Componentからパラメータを取得するサービスのリクエスト
                self.get_position = rospy.ServiceProxy(self.robotname +'/navi_get_param', navi_get_param1)
                g_position = self.get_position()

                #self.positionに目標値を格納
                self.position = g_position.target_position
                print(self.position)

                #できたらOKを返す
                re = "OK"
                return navi_get_parameterResponse(re,self.position)

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

                #取得できなかったら空の配列とERRORを返す
                self.position = [0.0]
                re ="ERROR"
                return navi_get_parameterResponse(re,self.position)
        else:
            print("get parameter is failed")
            self.text = " "
            re ="ERROR"
            return navi_get_parameterResponse(re,self.text)



    #Component_statusを取得するようの関数
    def component_status(self, co_name):
        try:
            state_name = self.robotname + '/get_state/' + self.component_ref
            get_component_status = rospy.ServiceProxy(state_name, component_status)
            return get_component_status(co_name)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return get_component_status(co_name)


    def add_item(self, _list, item):
        if item[1] not in _list:
            _list.append(item)
            print(f"{item} を追加しました")
            # print(_list)
            return True
        else:
            print(f"{item} は既にリストに存在します")
            print(_list)
            return False


    def remove_item(self, _list, item):
        if item  in _list:
            _list.remove(item)
            print(f"{item} を削除しました")
            # print(_list)
            return True
        else:
            print(f"{item} はリストにありません")
            return False

    def event_cb1(self,result):
        # if result.event_type == "speech_input_started" or result.event_type == "speech_input_finished" or result.event_type == "speech_recognized":
        if result.event_type == "speech_recognized":
            print(result.event_type)
            current_timestamp = result.timestamp
            
            if current_timestamp == self.last_timestamp:
                return
            _list = [result.event_type, result.timestamp, result.recognized_text]
            self.last_timestamp = current_timestamp
            

        self.event_count += 1
        self.event_id = "speech_recognition" + str(self.event_count)

        subscribe_id = self.subscribe_id
        
        if self.add_item(self.speech_recognition_list,[self.event_id, _list]):

            self.notify_event(self.event_id, result.event_type, subscribe_id)
    
    def event_cb2(self,result):
        print(result.event_type)
        if  result.event_type == "person_detected":
            current_timestamp = result.timestamp
            if current_timestamp == self.last_timestamp:
                exit
            _list = [result.event_type, result.timestamp,result.number]
            self.last_timestamp = current_timestamp


        self.event_count += 1
        self.event_id = "person_detected" + str(self.event_count)
        print(self.event_id)

        subscribe_id = self.subscribe_id
        if self.add_item(self.person_detected_list , [self.event_id,_list]):
            self.notify_event(self.event_id, result.event_type, subscribe_id)
    
    def event_cb3(self,result):
        
        if  result.event_type == "person_localized":
            current_timestamp = result.timestamp
            if current_timestamp == self.last_timestamp:
                exit
            _list = [result.event_type, result.timestamp,result.person_ref,result.position_data]
            self.last_timestamp = current_timestamp


        self.event_count += 1
        self.event_id = "person_localized" + str(self.event_count)

        subscribe_id = self.subscribe_id

        print(self.add_item(self.person_position_list ,[self.event_id,_list]))
        self.notify_event(self.event_id, result.event_type, subscribe_id)

    
    def event_cb4(self,result):
        
        if  result.event_type == "person_identified":
            current_timestamp = result.timestamp
            if current_timestamp == self.last_timestamp:
                exit
            _list = [result.event_type, result.timestamp, result.person_ref]
            self.last_timestamp = current_timestamp


        self.event_count += 1
        self.event_id = "person_identified_" + str(self.event_count)

        subscribe_id = self.subscribe_id

        print(self.add_item(self.person_identi_list ,[self.event_id,_list]))
        self.notify_event(self.event_id, result.event_type, subscribe_id)


    def notify_event(self,event_id, event_type, subscribe_id):
        rospy.loginfo(f'event_id:{event_id}, event_type:{event_type}, subscribe_id:{subscribe_id}')
        
        notify_event_pub = notifyevent()
        notify_event_pub.event_id = event_id
        notify_event_pub.event_type = event_type
        notify_event_pub.subscribe_id = subscribe_id
        print("pub event_id")
        self.notify_event_pub.publish(notify_event_pub)


    def event_recognized_cb(self,req):

        print(req.event_id)

        if "recog" in req.event_id:
            data_list = self.speech_recognition_list
            print(data_list)

            search_event_id = req.event_id

            for data in data_list:

                if data[0] == search_event_id:
                    print(data[0])
                    timestamp = data[1][1]
                    recognized_text = data[1][2]
                    print(f"timestamp: {timestamp}")
                    print(f"recognized_text: {recognized_text}")
                    return get_event_detail_speech_recognizedResponse(timestamp,recognized_text)
            else:
                print(f"{search_event_id}に一致するデータは見つかりませんでした。")
                return get_event_detail_speech_recognizedResponse("", "")



    def event_detected_cb(self,gr_req):
        if "detected" in gr_req.event_id:
            data_list = self.person_detected_list
            print(data_list)

            search_event_id = gr_req.event_id

            for data in data_list:

                # 文字列内に0番目の要素が含まれているかチェック
                if data[0] == search_event_id:
                    print(f"関連データ: {data[1][1]}")
                    
                    return get_event_detail_person_detectedResponse(data[1][1],data[1][2])


            else:
                print(f"{search_event_id}に一致するデータは見つかりませんでした。")     
                get_event_detail_person_detectedResponse("",0)   


    def event_local_cb(self,gr_req):
                

        if "local" in gr_req.event_id:
            data_list = self.person_position_list
            print(self.person_position_list)
            print(data_list)

            search_event_id = gr_req.event_id

            for data in data_list:
                key = data[0]  # 各配列の0番目の要素を取得
                
                # 文字列内に0番目の要素が含まれているかチェック
                if data[0] == search_event_id:
                    print(data[0])
                    print(f"関連データ: {data[1][1]}")
                    return get_event_detail_person_localizedResponse(data[1][1],data[1][2],data[1][3])
                    break  

            else:
                print(f"{search_event_id}に一致するデータは見つかりませんでした。")        
                return get_event_detail_person_localizedResponse("", "")


    def event_identified_cb(self,gr_req):
        print(gr_req.event_id)
                

        if "identi" in gr_req.event_id:
            data_list = self.person_identi_list
            print(self.person_identi_list)

            search_event_id = gr_req.event_id

            for data in data_list:
                key = data[0]  # 各配列の0番目の要素を取得
                
                # 文字列内に0番目の要素が含まれているかチェック
                if data[0] == search_event_id:
                    print(data[0])
                    print(f"関連データ: {data[1][2]}")
                    return get_event_detail_person_identifiedResponse(data[1][1],[data[1][2]])
                    break  

            else:
                print(f"{search_event_id}に一致するデータは見つかりませんでした。")        
                return get_event_detail_person_identifiedResponse("", [""])

        # except Exception as e:
        #     print(e)
        #     return get_event_detail_person_identifiedResponse("", [""])



    def subscribe_cb(self, sub_req):
        self.subcount += 1
        self.component_ref = self.BINDCOMP
        print(self.component_ref)
        self.event_type = sub_req.event_type
        #self.event_id = sub_req.event_type
        self.subscribe_id = sub_req.event_type + str(self.subcount)
        
        
        #アプリに送信するためのトピック通信(notify_event())
        self.notify_event_pub  = rospy.Publisher( '/notify_event', notifyevent,queue_size = 1)
        self.notify_event_pub1 = rospy.Publisher( '/notify_event_pub/recog', notifyevent,queue_size = 1)
        self.notify_event_pub2 = rospy.Publisher( '/notify_event_pub/detec', notifyevent,queue_size = 1)
        self.notify_event_pub3 = rospy.Publisher( '/notify_event_pub/local', notifyevent,queue_size = 1)
        self.notify_event_pub4 = rospy.Publisher( '/notify_event_pub/inden', notifyevent,queue_size = 1)


        #コンポーネントからの結果を受信するためのトピック通信
        self.event_subscriber1 = rospy.Subscriber( self.robotname + '/event_speechrec', notify_speechrec,self.event_cb1)
        self.event_subscriber2 = rospy.Subscriber( self.robotname + '/event_persondetec', notify_persondetect,self.event_cb2)
        self.event_subscriber3 = rospy.Subscriber( self.robotname + '/event_localization', notify_localrec,self.event_cb3)
        self.event_subscriber4 = rospy.Subscriber( self.robotname + '/event_identified', notify_identified,self.event_cb4)
        

        # print(sub_req)
        # if sub_set.result == "True":
        return subscribe_Response("OK",self.subscribe_id)
        # else :
        #     return subscribe_Response("ERROR",self.subscribe_id)



    def unsubscribe_cb(self, unsub_req):
        if self.subscribe_id == unsub_req.subscribe_id:
            print("AAA")


        try:
            print(f"unsubscribe: {unsub_req}")

            #topicの購読をやめる
            self.event_subscriber1.unregister()
            self.event_subscriber2.unregister()
            self.event_subscriber3.unregister()
            self.event_subscriber4.unregister()

            #アプリへ通知送信をやめる
            self.notify_event_pub1.unregister()
            self.notify_event_pub2.unregister()
            self.notify_event_pub3.unregister()
            self.notify_event_pub4.unregister()

            return unsubscribe_Response("OK")

        except:
            return unsubscribe_Response("ERROR")


    # execute()を要求されたときの関数
    def execute_cb(self, exe_req):
        print(f"{exe_req.command_unit_list} {self.component_ref}")
        
        #要求されたcommand
        self.command = exe_req.command_unit_list

        goal = executeGoal()


        
        try:

            if self.command == "start":
                self.component_ref = self.BINDCOMP
                print(self.component_ref)
                #コンポーネントの状態をとるサービス通信
                self.c_status = self.component_status(self.component_ref) 
                print(f"{self.component_ref} is {self.c_status.status}")
                
                #コンポーネントがREADYなら

                if self.c_status.status == "READY":

            
                    try:
                        #コンポーネントのstart()メソッドを指定する
                        goal.command_name = self.command
                        print(goal.command_name)

                        #アクションリクエストを送信
                        self.client.send_goal(goal, done_cb=self.action_done_callback)

                        #メッセージを遅れたらOKを返答
                        return_t ="OK"
                        
                        #Returncode_t
                        return executeResponse(return_t)
                    except Exception as e :
                        return_t ="ERROR"
                        print("aaa")
                        rospy.logerr("Connection failed: %s", str(e)) 
                        return executeResponse(return_t)

                #コンポーネントが準備完了ではないなら
                else:
                    
                    rospy.loginfo(f"This component is {self.c_status.status}")
                    return_t ="ERROR"
                    #Returncode_t
                    return executeResponse(return_t)

            elif self.command == "stop":
                print("stop")
                try:
                    goal.command_name = self.command #コンポーネントのstop()メソッドを指定する
                    self.client.send_goal(goal, done_cb=self.action_done_callback)#アクションリクエストを送信

                    return_t ="OK"
                    #Returncode_t
                    return executeResponse(return_t)
                except:
                    return_t ="ERROR"
                    #Returncode_t
                    return executeResponse(return_t)
                    
            elif self.command == "suspend":
                print("suspend")

                try:
                    goal.command_name = self.command #コンポーネントのsuspend()メソッドを指定する
                    self.client.send_goal(goal, done_cb=self.action_done_callback)#アクションリクエストを送信

                    return_t ="OK"
                    #Returncode_t
                    return executeResponse(return_t)

                except:
                    return_t ="ERROR"
                    #Returncode_t
                    return executeResponse(return_t)            
            
            elif  self.command == "resume":
                print("resume")
                try:
                    goal.command_name = self.command #コンポーネントのsuspend()メソッドを指定する
                    self.client.send_goal(goal, done_cb=self.action_done_callback)#アクションリクエストを送信

                    return_t ="OK"
                    return executeResponse(return_t)#Returncodeを返答                    

                except Exception as e :
                    rospy.logerr("Connection failed: %s", str(e)) 
                    return_t ="ERROR"
                    #Returncode_t
                    return executeResponse(return_t)
            
            else:
                return_t ="ERROR"
                #Returncode_t
                return executeResponse(return_t)

        except Exception as e :
            return_t ="ERROR"
            print("aaa")
            rospy.logerr("Connection failed: %s", str(e)) 
            print("finish")
            return executeResponse(return_t)


    def action_done_callback(self, cb_status, cb_result):# アクションが完了したら呼び出される
        if cb_status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Action succeeded with result: {cb_result.success}")
        else:
            rospy.loginfo(f"Action failed or was canceled with state: {cb_status}")

    #Componentからの完了通知が来たらApp.に送信
    def completed(self,c_req):
        self.command_count += 1
        command_id = c_req.command_id + str(self.command_count)
        status = c_req.status
        rospy.loginfo(f"command_id {command_id} is {status}")

        
        _list = [command_id, status]
        

        if self.add_item(self.command_list , _list):
            completed_Pub = completed()
            completed_Pub.command_id = command_id
            completed_Pub.status = status
            self.completed_pub.publish(completed_Pub)


    def get_command_result_cb(self,req):
        print("get_command_result")

        data_list = self.command_list
        print(data_list)

        search_command_id = req.command_id

        for data in data_list:
            if data[0] == search_command_id:
                command_id = data[0]
                result = data[1]
                print(f"command_id: {command_id}")
                print(f"result: {result}")
                return get_command_resultResponse(result)
        else:
            print(f"{search_command_id}に一致するデータは見つかりませんでした。")
            return get_command_resultResponse("")


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

    node_name = robot + '_Engine'

    rospy.init_node(node_name)
    try:
        _service = EngineService(robotname)
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
        print("finish")
        exit

