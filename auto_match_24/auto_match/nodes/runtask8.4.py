#!/usr/bin/python3

import sys
import os
import yaml
import _thread
import threading
import pickle

import rospy
import rospkg
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseResult
import common.msg
import common.srv
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction
# import common.action
import swiftpro.msg
from std_msgs.msg import String
from swiftpro.msg import position
from vision_msgs.msg import Detection2DArray
from collections import Counter
cube_list0 = []
cube_list = []
cube_list2 = []
a = 0
b = 0
c = 0
def cube_list_callback(data):
    global cube_list
    global cube_list0
    cube_list = []  # Çå¿Õ cube_list
    
    for obj in data.detections:
        obj_id = obj.results[0].id
        obj_pose = [obj.results[0].pose.pose.position.x, obj.results[0].pose.pose.position.y, obj.results[0].pose.pose.position.z]
        cube_list.append([obj_id, obj_pose])

    #print(cube_list)
    cube_list0 = cube_list
    return cube_list
class SwiftProInterface:
    def __init__(self):
        # 创建控制机械臂的topic发布者
        self.arm_position_pub = rospy.Publisher(
            "position_write_topic", swiftpro.msg.position, queue_size=1)   # 机械臂运动位置发布者
        self.arm_pump_pub = rospy.Publisher(
            "pump_topic", swiftpro.msg.status, queue_size=1)               # 机械臂气泵状态发布者
        self.arm_status_pub = rospy.Publisher(
            "swiftpro_status_topic", swiftpro.msg.status, queue_size=1)    # 机械臂开关状态发布者


    def set_pose(self, x, y, z):
        '''
        发布机械臂运动位置
        '''
        pos = position()
        pos.x = x
        pos.y = y
        pos.z = z
        # rospy.loginfo(f"set pose {x},{y},{z}")
        self.arm_position_pub.publish(pos)
        rospy.sleep(1)

    def set_pump(self, enable:bool):
        '''
        吸取或释放，设定机械臂气泵状态
        '''
        rospy.loginfo(f" 设定机械臂气泵状态为：{enable}")
        if enable:
            self.arm_pump_pub.publish(swiftpro.msg.status(1))
        else:
            self.arm_pump_pub.publish(swiftpro.msg.status(0))

    def set_status(self, lock:bool):
        '''
        设定机械臂开关状态
        '''
        rospy.loginfo(f"set arm status {lock}")
        if lock:
            self.arm_status_pub.publish(swiftpro.msg.status(1))
        else:
            self.arm_status_pub.publish(swiftpro.msg.status(0))



class CamAction:
    def __init__(self):
        # 获取标定文件相关信息
        rospack = rospkg.RosPack()
        package_path = os.path.join(rospack.get_path('auto_match'))          # 获取功能包路径
        items_path = os.path.join(package_path, 'config', 'items_config.yaml')  # 获取物体标签路径
        try:
            with open(items_path, "r", encoding="utf8") as f:
                items_content = yaml.load(f.read(), Loader=yaml.FullLoader)
        except Exception:
            rospy.logerr("can't not open file")
            sys.exit(1)
        if isinstance(items_content, type(None)):
            rospy.logerr("items file empty")
            sys.exit(1)

        # 根据yaml文件，确定抓取物品的id号
        self.search_id = [
            items_content["items"][items_content["objects"]["objects_a"]]
        ]
        self.search_id1 = [
            items_content["items"][items_content["objects"]["objects_b"]]
        ]
        self.search_id2 = [
            items_content["items"][items_content["objects"]["objects_c"]]
        ]

    def detector(self):
        '''
        获取需要抓取的物品在显示屏上的坐标位置
        @return: 需要抓取的物品列表cube_list

        cube_list[i]:代表第几个物体
        cube_list[i][0]:代表第i个物体的ID信息;               cube_list[i][1]:代表第i个物体的位置信息
        cube_list[i][1][1]:代表第i个物体的x方向上的位置;     cube_list[i][1][2:代表第i个物体的y方向上的位置
        '''
        obj_dist = {}
        cube_list = []
        obj_array = None

        try:
            obj_array = rospy.wait_for_message(
                "/objects", Detection2DArray, timeout=5)
        except Exception:
            cube_list.clear()
            return cube_list
        
        # # 提取
        # for obj in obj_array.detections:
        #     obj_dist[obj.results[0].id] = [obj.bbox.center.x, obj.bbox.center.y, 0]

        # # 筛选出需要的物品 cube_list中的key代表识别物体的ID，value代表位置信息
        # for key, value in obj_dist.items():
        #     if key in self.search_id:
        #         cube_list.append([key, value])
        # for key, value in obj_dist.items():
        #     if key in self.search_id1:
        #         cube_list.append([key, value])
        # for key, value in obj_dist.items():
        #     if key in self.search_id2:
        #         cube_list.append([key, value])                
        # return cube_list


class ArmAction:
    def __init__(self):
        self.cam = CamAction()
        try: self.robot = RobotMoveAction()
        except Exception as e:  print("except robot:",e)
        print("========实例化Robot===== ")
        # 获取标定文件数据
        filename = os.environ['HOME'] + "/thefile.txt"
        with open(filename, 'r') as f:
            s = f.read()
        arr = s.split()
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]        

        # 创建机械臂控制接口的对象
        self.interface = SwiftProInterface()
        self.stop_flag = False  # 任务的启停标志
        self.grasp_status_pub = rospy.Publisher("/grasp_status", String, queue_size=1)
    def detectorD(self):
        global cube_list0
        global object_list
        global a
        global cube_list2
        cube_list2 = []
        cube_list4 = []
        if self.stop_flag: return
        r1 = rospy.Rate(0.25)
        rospy.sleep(2)
        cube_list1 = cube_list0
        cube_list1 = sorted(cube_list1, key=lambda y: y[1][1], reverse=True)
        #print(cube_list1)                                ##提取第一行
        for i in range(len(cube_list1)):
            if cube_list1[i][1][1] - cube_list1[i+1][1][1] < 53:
                cube_list2.append(cube_list1[i])
            else:
                cube_list2.append(cube_list1[i])
                break
        print(cube_list2)
        for i in range(len(cube_list2)):                      ##提取第一行D的个数
            if cube_list2[i][0] == object_list[2][0]:
                a += 1
                cube_list4.append(cube_list2[i])
        # # if cube_list1[0][0] == object_list[2][0] and cube_list1[0][1][1] >= 230:
        # #     a += 1
        # #     cube_list2 = cube_list1[0]  
        # if cube_list4 == []:
        #        cube_list4.append([5,[122,0,0]])
        return cube_list4
    def detectorC(self):
        global cube_list0
        global object_list
        global a
        global cube_list2
        cube_list2 = []
        cube_list4 = []
        if self.stop_flag: return
        r1 = rospy.Rate(0.25)
        rospy.sleep(2)
        cube_list1 = cube_list0
        cube_list1 = sorted(cube_list1, key=lambda y: y[1][1], reverse=True)
        #print(cube_list1)

        for i in range(len(cube_list1) - 1):
            if cube_list1[i][1][1] - cube_list1[i+1][1][1] < 53:
                cube_list2.append(cube_list1[i])
            else:
                cube_list2.append(cube_list1[i])
                break
        print(cube_list2)
        for i in range(len(cube_list2)):
            if cube_list2[i][0] == object_list[1][0]:
                a += 1
                cube_list4.append(cube_list2[i])
        # if cube_list1[0][0] == object_list[2][0] and cube_list1[0][1][1] >= 230:
        #     a += 1
        #     cube_list2 = cube_list1[0] 
        if cube_list4 == []:
               cube_list4.append([5,[122,0,0]])   
        return cube_list4 
    def detectorB(self):
        global cube_list0
        global object_list
        global a
        global cube_list2
        cube_list2 = []
        cube_list4 = []
        if self.stop_flag: return
        r1 = rospy.Rate(0.25)
        rospy.sleep(2)
        cube_list1 = cube_list0
        cube_list1 = sorted(cube_list1, key=lambda y: y[1][1], reverse=True)
        #print(cube_list1)
        for i in range(len(cube_list1) - 1):
            if cube_list1[i][1][1] - cube_list1[i+1][1][1] < 53:
                cube_list2.append(cube_list1[i])
            else:
                cube_list2.append(cube_list1[i])
                break
        print(cube_list2)
        for i in range(len(cube_list2)):
            if cube_list2[i][0] == object_list[0][0]:
                a += 1
                cube_list4.append(cube_list2[i])
        if cube_list4 == []:
               cube_list4.append([5,[122,0,0]])
        return cube_list4
    def detectorend(self):
        global cube_list0
        global object_list
        global a
        global cube_list2
        cube_list2 = []
        cube_list4 = []
        cube_list5 = []
        #self.robot.step_back()  # 后退
        #if self.stop_flag: return
        r1 = rospy.Rate(0.25)
        rospy.sleep(2)
        cube_list1 = cube_list0
        cube_list1 = sorted(cube_list1, key=lambda y: y[1][1], reverse=True)
        #print(cube_list1)
        if len(cube_list1) != 0:
            if cube_list1[0][1][1] > 230:
                if len(cube_list1) == 1:
                    return cube_list1
                else:
                    i = 0
                    while i < len(cube_list1) - 1:
                        if cube_list1[i][1][1] - cube_list1[i + 1][1][1] < 53:
                            cube_list2.append(cube_list1[i])
                        else:
                            cube_list2.append(cube_list1[i])
                            print(cube_list2)
                            break
                        i += 1

                    return cube_list2
            else :
                return cube_list5
        else :
            return cube_list5
    def detectorend1(self):
        global cube_list0
        global object_list
        global a
        global cube_list2
        cube_list2 = []
        cube_list4 = []
        cube_list5 = []
        #self.robot.step_back()  # 后退
        #if self.stop_flag: return
        r1 = rospy.Rate(0.25)
        rospy.sleep(2)
        cube_list1 = cube_list0
        cube_list1 = sorted(cube_list1, key=lambda y: y[1][1], reverse=True)
        #print(cube_list1)
        while True:
            cube_list1 = cube_list0
            cube_list1 = sorted(cube_list1, key=lambda y: y[1][0], reverse=True)
            if cube_list1[0][1][1] < 240 or cube_list1[0][1][1] > 280:
                if cube_list1[0][1][1] < 240:
                    print("前进")
                    self.robot.step_go1()
                    rospy.sleep(1)
                    if self.stop_flag: return
                elif cube_list1[0][1][1] > 280:
                    print("后退")
                    self.robot.step_back()
                    rospy.sleep(1)
                    if self.stop_flag: return
            else:
                break
        #rospy.sleep(2)
        cube_list1 = cube_list0
        cube_list1 = sorted(cube_list1, key=lambda x: x[1][0], reverse=True)
        if len(cube_list1) != 0:
            return cube_list1
        else :
            return cube_list5      	
    def grasp(self,name):
        '''
        使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
        @return: 抓取到物品的id, 0为未识别到需要的物品
        '''
        # global cube_list0
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(0.25)
        # rospy.sleep(2)
        # cube_list1 = cube_list0
        # cube_list1 = sorted(cube_list1, key=lambda y: y[1][1], reverse=True)

        #print(cube_list1)
        #cube_list2.append([cube_list1[0][0], cube_list1[0][1]])
        #cube_list2.append([cube_list1[1][0], cube_list1[1][1]])
        #cube_list2.append([cube_list1[2][0], cube_list1[2][1]])
        #print(cube_list)
        #cube_list = sorted(cube_list, key=lambda x: x[1][1], reverse=True)
        #print(cube_list)
        # rospy.sleep(2)
        #cube_list1 = sorted(cube_list1, key=lambda x: x[1][1], reverse=True)
        if len(name) == 0:
            rospy.logwarn("没有找到物品啊。。。去下一个地方")
            self.grasp_status_pub.publish(String("1"))
            return 0

        #cube_list1 = sorted(cube_list1, key=lambda x: x[1][1], reverse=True)
        #print(cube_list1)
        # 获取机械臂目标位置
        x = self.x_kb[0] * (name[0][1][1]-13)+ self.x_kb[1]
        y = self.y_kb[0] * (name[0][1][0]) + self.y_kb[1]
        z = -55
        self.interface.set_pose(150, 0, 10)
        print(f"找到物品了！它在: {x}, {y}, {z}")
        rospy.sleep(0.5)
        # 机械臂移动到目标位置上方
        self.interface.set_pose(x, y, z + 20)
        #rospy.sleep(1)

        # 打开气泵，进行吸取
        self.interface.set_pump(True)
        #rospy.sleep(1)

        # 机械臂移动到目标位置
        self.interface.set_pose(x, y, z)
        #rospy.sleep(1)

        
        #r2.sleep()

        # 抬起目标方块
        print(f"我把物品抬起来了")
        self.interface.set_pose(x, y, z + 120)
        #rospy.sleep(2)
        #r1.sleep()

        self.grasp_status_pub.publish(String("0"))

        return True

    def drop1(self, check=False):
        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(0.25)
        x = 300
        y = -5
        z = -20
        cube_list3 = []
        global cube_list0
        # 默认放置位置
        rospy.sleep(1)
        self.interface.set_pose(x, y, z)
        rospy.sleep(2)

        # 关闭气泵
        self.interface.set_pump(0)
        #r2.sleep()
        #rospy.sleep(2)

        # self.interface.set_pose(300, 0, 120)
        #r1.sleep()
        self.arm_grasp_ready()  # 移动机械臂到其他地方
        #rospy.sleep(2)

        self.grasp_status_pub.publish(String("0"))

        return True
    def drop2(self, check=False):
        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(0.25)
        x = 300
        y = 0
        z = 80
        cube_list3 = []
        global cube_list0
        if cube_list0 != []:
            # 控制机械臂移动到其他地方，以免挡住摄像头
            self.interface.set_pose(0, 225, 160)
            # r1.sleep()
            rospy.sleep(1)
            cube_list3 = cube_list0
            cube_list3 = sorted(cube_list3, key=lambda x: x[1][0], reverse=True)
            if len(cube_list3) > 0:
                x = self.x_kb[0] * cube_list3[0][1][1] + self.x_kb[1]
                y = self.y_kb[0] * cube_list3[0][1][0] + self.y_kb[1]
                z = 66
 
        # 默认放置位置
        self.interface.set_pose(220, 0, 172) 
        self.interface.set_pose(x, y, z)
        rospy.sleep(1)


        # 关闭气泵
        self.interface.set_pump(0)
        #r2.sleep()
        #rospy.sleep(2)

        # self.interface.set_pose(300, 0, 120)
        #r1.sleep()
        self.arm_grasp_ready()  # 移动机械臂到其他地方
        #rospy.sleep(2)

        self.grasp_status_pub.publish(String("0"))

        return True
    def drop3(self, check=False):
        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 300
        y = 0
        z = 140
        cube_list3 = []
        global cube_list0
        if cube_list0 != []:
            # 控制机械臂移动到其他地方，以免挡住摄像头
            self.interface.set_pose(0, 225, 160)
            # r1.sleep()
            rospy.sleep(1)
            cube_list3 = cube_list0
            cube_list3 = sorted(cube_list3, key=lambda x: x[1][0], reverse=True)
            if len(cube_list3) > 0:
                x = self.x_kb[0] * cube_list3[0][1][1] + self.x_kb[1]
                y = self.y_kb[0] * cube_list3[0][1][0] + self.y_kb[1]
                z = 66
    
        # 默认放置位置
        #self.interface.set_pose(70, 0, 60)
        self.interface.set_pose(220, 0, 180) 
        self.interface.set_pose(x, y, z)
        rospy.sleep(1)


        # 关闭气泵
        self.interface.set_pump(0)
        self.arm_grasp_ready()  # 移动机械臂到其他地方
        self.grasp_status_pub.publish(String("0"))

        return True
    def arm_position_reset(self):
        '''
        校准机械臂的坐标系, 机械臂因碰撞导致坐标计算出问题时使用
        '''
        r1 = rospy.Rate(10)
        self.interface.set_status(False)
        r1.sleep()
        self.interface.set_status(True)
        r1.sleep()

    def arm_home(self, block=False):
        '''
        收起机械臂(无物品)
        '''
        self.interface.set_pose(130, 180, 35)
        if block:
            rospy.sleep(0.5)

    def arm_grasp_ready(self, block=False):
        '''
        移动机械臂到摄像头看不到的地方，以方便识别与抓取
        '''
        self.interface.set_pose(10, 180, 160)  
        if block:
            rospy.sleep(0.5)

    def arm_grasp_laser(self, block=False):
        self.interface.set_pose(160, 0, 20)
        if block:
            rospy.sleep(0.5)

class RobotMoveAction:
    def __init__(self):
        # 创建控制spark直走的action客户端
        self.move_action_cli = actionlib.SimpleActionClient(
            'move_straight', MoveStraightDistanceAction)
        self.move_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        # 创建控制spark旋转的action客户端
        self.turn_action_cli = actionlib.SimpleActionClient(
            'turn_body', TurnBodyDegreeAction)
        self.turn_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))


        # 创建获取spark前后距离的service客户端
        rospy.wait_for_service('/get_distance')
        self.distance_srv = rospy.ServiceProxy(
            'get_distance', common.srv.GetFrontBackDistance)

        # 创建导航地点的话题发布者
        self.goto_local_pub = rospy.Publisher(
            "mark_nav", String, queue_size=1)

    def goto_local(self, name):
        '''
        根据目标点名称,发布目标位置到MoveBase服务器,根据返回状态进行判断
        @return: True 为成功到达, False 为失败
        '''

        # 发布目标位置
        self.goto_local_pub.publish("go "+name)

        # 设定1分钟的时间限制，进行阻塞等待
        try:
            ret_status = rospy.wait_for_message(
                'move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
        except Exception:
            rospy.logwarn("nav timeout!!!")
            ret_status = GoalStatus.ABORTED

        # 如果一分钟之内没有到达，放弃目标
        if ret_status != GoalStatus.SUCCEEDED:
            rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
                GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
            try:
                rospy.wait_for_message(
                    'move_base/result', MoveBaseActionResult, rospy.Duration(3))
            except Exception:
                rospy.logwarn("move_base result timeout. this is abnormal.")
            rospy.loginfo("==========Timed out achieving goal==========")
            return False
        else:
            rospy.loginfo("==========Goal succeeded==========")
            return True
    
    def step_back(self):
        '''
        后退, 用于抓取或放置后使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=-0.04,
                move_distance=0.005,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True

    def step_back1(self):
        '''
        后退, 用于抓取或放置后使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=-0.2,
                move_distance=0.50,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True    
    def step_turn(self, angle):
        '''
        Ðý×ª, ÓÃÓÚµ÷Õû³¯Ïò
        @param angle: Ðý×ª½Ç¶È, ÕýÊý±íÊ¾Ë³Ê±ÕëÐý×ª, ¸ºÊý±íÊ¾ÄæÊ±ÕëÐý×ª
        @return: True Îªµ÷Õû³É¹¦, False Îªµ÷ÕûÊ§°Ü
        '''
        self.turn_action_cli.send_goal_and_wait(
            common.msg.TurnBodyDegreeGoal(
                const_rot_vel=50,
                goal_degree=angle,
            ),
            rospy.Duration.from_sec(5)  # ³¬¹ý5sÎª³¬Ê±
        )
        return True
    def step_go(self,dis):
        '''
        前进, 用于抓取或放置前使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=0.8,
                move_distance=dis,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True
    def step_go1(self):
        '''
        前进, 用于抓取或放置前使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=0.04,
                move_distance=0.05,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True
    def step_go2(self,dst):
        '''
        前进, 用于抓取或放置前使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=0.04,
                move_distance=dst,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True
class AutoAction:
    def __init__(self):
        # # 初始化节点
        # if init_node:
        rospy.init_node('spark_auto_match_node', anonymous=True)

        print("========ready to task===== ")

        # 实例化Cam
        try: self.cam = CamAction()
        except Exception as e:  print("except cam:",e)
        print("========实例化Cam===== ")
        # 实例化Arm
        try: self.arm = ArmAction()
        except Exception as e:  print("except arm:",e)
        print("========实例化Arm===== ")
        # 实例化Robot
        try: self.robot = RobotMoveAction()
        except Exception as e:  print("except robot:",e)
        print("========实例化Robot===== ")

        # 订阅任务控制指令的话题
        self.task_cmd_sub = rospy.Subscriber("/task_start_flag", String, self.task_cmd_cb) # 订阅任务开始与否信号
        self.task_run_th = threading.Thread(target=lambda: "pass") # 创建线程对象
        self.stop_flag = False  # 任务的启停标志

        # 订阅机械臂手动控制的话题
        self.grasp_sub = rospy.Subscriber("grasp", String, self.grasp_cb)

        rospy.loginfo("spark_auto_match_node is ready")

    # 接收到启动自动赛信号，开始执行任务
    def run_task(self):
        global cube_list0
        global object_list
        global cube_list2
        global a
        global b
        global c
        yi = 0
        san = 0
        wu = 0
        qi = 0
        d = 0
        e = 0
        f = 0
        ret = False # 是否导航成功标志
        item_type = 0 
        self.arm.arm_home()  # 移动机械臂到其他地方
        self.robot.step_turn(95)
        if self.stop_flag: return
        self.robot.step_go(0.4)
        if self.stop_flag: return 
        self.robot.step_turn(-60)
        if self.stop_flag: return
        self.robot.step_go(1.0)
        if self.stop_flag: return 
        ret = self.robot.goto_local("Collection_B") # 根据抓到的物品类型，导航到对应的放置区
        if self.stop_flag: return  
        self.robot.step_turn(-150)
        if self.stop_flag: return
        self.robot.step_go(0.6)
        if self.stop_flag: return 
        ret = self.robot.goto_local("wu") # 根据抓到的物品类型，导航到对应的放置区
        if self.stop_flag: return  
 
        exit()
        # ===== 现在开始执行任务 =====
        rospy.loginfo("start task now.")
        # ==== 离开起始区,避免在膨胀区域中，导qqqqq 致导航失败 =====
        self.robot.step_go(1.2)
        if self.stop_flag: return	
        self.robot.step_turn(-100)
        if self.stop_flag: return		
        # ==== 移动机械臂 =====
        self.arm.arm_position_reset()  # 重置机械臂坐标系
        self.arm.arm_grasp_ready()  # 移动机械臂到其他地方

        # ===== 导航到分类区 =====
        ret = self.robot.goto_local('fenlei')
        if self.stop_flag: return
        rospy.sleep(1)
        object_list = cube_list0
        #rospy.sleep(1)
        #print(object_list)
        object_list = sorted(object_list, key=lambda x: x[1][0], reverse=False)
        print(object_list)
        sorting_time = 0  # 该变量定义了在同一地点的抓取次数
        # while sorting_time < 3:
        self.robot.step_turn(180)
        if self.stop_flag: return
        ret = self.robot.goto_local('yi')
        if self.stop_flag: return
    
##在一点识别B
        if ret: 
            item_type = self.arm.detectorB()
            print("在yi点抓B")
            b = a
        while b > 0 :
            # self.robot.step_back()  # 后退
            # if self.stop_flag: return
            if b == 3:
                print("3个B")
                item_type = self.arm.detectorB()
            elif b == 2:
                print("2个B")
                item_type = self.arm.detectorB()
            elif b == 1:
                print("1个B")
                item_type = self.arm.detectorB()
            if len(cube_list2) == 1:
                yi = 1
            print("========扫描中，准备抓取===== ")
            self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
            print("========向后退一点===== ")
            self.robot.step_back()  # 后退
            if self.stop_flag: return
            self.arm.arm_grasp_ready()
            print("========前往放置区===== ")
            self.robot.step_turn(-90)
            if self.stop_flag: return
            self.robot.step_go(0.2)
            if self.stop_flag: return 
            self.robot.step_turn(80)
            if self.stop_flag: return
            self.robot.step_go(0.8)
            if self.stop_flag: return 
            ret = self.robot.goto_local("Collection_B") # 根据抓到的物品类型，导航到对应的放置区
            #rospy.sleep(2) # 停稳
            if self.stop_flag: return 
            #self.robot.step_go1()
            #if self.stop_flag: return        
            if ret:
                if d == 0:
                    self.arm.drop1()  # 放下物品
                    d += 1
                elif d == 1:
                    self.arm.drop2()  # 放下物品
                    d += 1
                elif d == 2:
                    self.arm.drop3()  # 放下物品
                    d += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            else:
                rospy.logerr("task error: navigation to the drop_place fails")
                if d == 0:
                    self.arm.drop1()  # 放下物品
                    d += 1
                elif d == 1:
                    self.arm.drop2()  # 放下物品
                    d += 1
                elif d == 2:
                    self.arm.drop3()  # 放下物品
                    d += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            b = b - 1
            if yi != 1:
                print("返回yi点")
                self.robot.step_turn(-120)
                if self.stop_flag: return
                self.robot.step_go(0.8)
                if self.stop_flag: return
                self.robot.step_turn(-90)
                if self.stop_flag: return
                self.robot.step_go(0.2)
                if self.stop_flag: return
                ret = self.robot.goto_local('yi')  
            else:
                print("去san点")
                self.robot.step_turn(120)
                if self.stop_flag: return
                self.robot.step_go(0.8)
                if self.stop_flag: return
                self.robot.step_turn(90)
                if self.stop_flag: return
                self.robot.step_go(0.2)
                if self.stop_flag: return   
                ret = self.robot.goto_local('san')        
            if self.stop_flag: return  
#在yi点抓D    
        b = 0
        a = 0
        c = 0
##在一点识别D
        if ret and yi != 1:       	
            item_type = self.arm.detectorD()
            print("在yi点识别D")
            b = a
        while b > 0 and yi != 1:
            # self.robot.step_back()  # 后退
            # if self.stop_flag: return
            if b == 3 and ret:
                print("3个D")
                item_type = self.arm.detectorD()
            elif b == 2 and ret:
                print("2个D")
                item_type = self.arm.detectorD()
            elif b == 1 and ret:
                print("1个D")
                item_type = self.arm.detectorD()
            if len(cube_list2) == 1:
                yi = 1
            print("========扫描中，准备抓取===== ")
            self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
            print("========向后退一点===== ")
            self.robot.step_back()  # 后退
            if self.stop_flag: return
            self.arm.arm_grasp_ready()
            print("========前往放置区===== ")
            self.robot.step_turn(120)
            if self.stop_flag: return
            self.robot.step_go(0.8)
            if self.stop_flag: return
            ret = self.robot.goto_local("Collection_D") # 根据抓到的物品类型，导航到对应的放置区
            #rospy.sleep(2) # 停稳
            if self.stop_flag: return 
            # self.robot.step_go1()
            if self.stop_flag: return        
            if ret:
                if f == 0:
                    self.arm.drop1()  # 放下物品
                    f += 1
                elif f == 1:
                    self.arm.drop2()  # 放下物品
                    f += 1
                elif f == 2:
                    self.arm.drop3()  # 放下物品
                    f += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            else:
                rospy.logerr("task error: navigation to the drop_place fails")
                if f == 0:
                    self.arm.drop1()  # 放下物品
                    f += 1
                elif f == 1:
                    self.arm.drop2()  # 放下物品
                    f += 1
                elif f == 2:
                    self.arm.drop3()  # 放下物品
                    f += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            b = b - 1
            c = 1
            if b != 0:
                print("返回yi点")
                self.robot.step_turn(180)
                if self.stop_flag: return
                self.robot.step_go(0.8)
                if self.stop_flag: return
                ret = self.robot.goto_local('yi')
                if self.stop_flag: return   
##若没有D则顺路抓C    
        if b == 0 and c == 0 and yi != 1:   #为达成上述循环与此if执行一个的效果，定义C标志位，来实现此效果。
            # self.robot.step_back()  # 后退
            # if self.stop_flag: return
            item_type = self.arm.detectorC() 
            if len(cube_list2) == 1:
                yi = 1
            print("在yi点若没有D则顺路抓C")
            if item_type[0][0] == object_list[1][0]:
                print("========扫描中，准备抓取===== ")
                self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
                print("========向后退一点===== ")
                self.robot.step_back()  # 后退
                if self.stop_flag: return
                self.arm.arm_grasp_ready()
                print("========前往放置区===== ")
                ret = self.robot.goto_local('ery')
                if self.stop_flag: return
                ret = self.robot.goto_local("Collection_C") # 根据抓到的物品类型，导航到对应的放置区
                #rospy.sleep(2) # 停稳
                if self.stop_flag: return 
                # self.robot.step_go1()
                if self.stop_flag: return        
                if ret:
                    if e == 0:
                        self.arm.drop1()  # 放下物品
                        e += 1
                    elif e == 1:
                        self.arm.drop2()  # 放下物品
                        e += 1
                    elif e == 2:
                        self.arm.drop3()  # 放下物品
                        e += 1
                    else:
                        self.arm.drop()  #  d += 1
                    self.robot.step_back1()  # 后退
                    self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                    if self.stop_flag: return
                else:
                    rospy.logerr("task error: navigation to the drop_place fails")
                    if e == 0:
                        self.arm.drop1()  # 放下物品
                        e += 1
                    elif e == 1:
                        self.arm.drop2()  # 放下物品
                        e += 1
                    elif e == 2:
                        self.arm.drop3()  # 放下物品
                        e += 1
                    else:
                        self.arm.drop()  #  d += 1
                    self.robot.step_back1()  # 后退
                    self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            else:
                print("返回er点")
                self.robot.step_turn(100)
                if self.stop_flag: return
                self.robot.step_go(0.3)
                if self.stop_flag: return
                self.robot.step_turn(-90)
                if self.stop_flag: return 

        print("去san点") 
        self.robot.step_turn(-120)
        if self.stop_flag: return
        self.robot.step_go(0.5)
        if self.stop_flag: return
        self.robot.step_turn(-90)
        if self.stop_flag: return                
        ret = self.robot.goto_local('san')
        if self.stop_flag: return   
#在san点抓D    
        b = 0
        a = 0
        c = 0
        if ret: 
            item_type = self.arm.detectorD()
            print("在san点抓D")
            b = a
            print(b)
        while b > 0 and san != 1:
            # self.robot.step_back()  # 后退
            # if self.stop_flag: return
            if b == 3:
                print("3个D")
                item_type = self.arm.detectorD()
            elif b == 2:
                print("2个D")
                item_type = self.arm.detectorD()
            elif b == 1:
                print("1个D")
                item_type = self.arm.detectorD()
            if len(cube_list2) == 1:
                san = 1           
            print("========扫描中，准备抓取===== ")
            self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
            print("========向后退一点===== ")
            self.robot.step_back()  # 后退
            if self.stop_flag: return
            self.arm.arm_grasp_ready()
            print("========前往放置区===== ")
            ret = self.robot.goto_local("Collection_D") # 根据抓到的物品类型，导航到对应的放置区
            #rospy.sleep(2) # 停稳
            if self.stop_flag: return 
            # self.robot.step_go1()
            if self.stop_flag: return        
            if ret:
                if f == 0:
                    self.arm.drop1()  # 放下物品
                    f += 1
                elif f == 1:
                    self.arm.drop2()  # 放下物品
                    f += 1
                elif f == 2:
                    self.arm.drop3()  # 放下物品
                    f += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            else:
                rospy.logerr("task error: navigation to the drop_place fails")
                if f == 0:
                    self.arm.drop1()  # 放下物品
                    f += 1
                elif f == 1:
                    self.arm.drop2()  # 放下物品
                    f += 1
                elif f == 2:
                    self.arm.drop3()  # 放下物品
                    f += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            b = b - 1
            if san != 1:
                print("返回san点")
                ret = self.robot.goto_local('san')
                if self.stop_flag: return 
            else:
                ret = self.robot.goto_local('six')
                if self.stop_flag: return 
                ret = self.robot.goto_local('wu')
                if self.stop_flag: return 
            #rospy.sleep(1)
            #if self.stop_flag: return      
        b = 0
        a = 0
        c = 0
#在san点抓C
        if ret and san != 1: 
            item_type = self.arm.detectorC()
            print("在san点抓C")
            b = a      
        while b > 0 and san != 1:
            # self.robot.step_back()  # 后退
            # if self.stop_flag: return
            if b == 3:
                print("3个C")
                item_type = self.arm.detectorC()
            elif b == 2:
                print("2个C")
                item_type = self.arm.detectorC()
            elif b == 1:
                print("1个C")
                item_type = self.arm.detectorC()
            if len(cube_list2) == 1:
                san = 1
            print("========扫描中，准备抓取===== ")
            self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
            print("========向后退一点===== ")
            self.robot.step_back()  # 后退
            if self.stop_flag: return
            self.arm.arm_grasp_ready()
            print("========前往放置区===== ")
            ret = self.robot.goto_local("Collection_C") # 根据抓到的物品类型，导航到对应的放置区
            #rospy.sleep(2) # 停稳
            if self.stop_flag: return 
            # self.robot.step_go1()
            if self.stop_flag: return        
            if ret:
                if e == 0:
                    self.arm.drop1()  # 放下物品
                    e += 1
                elif e == 1:
                    self.arm.drop2()  # 放下物品
                    e += 1
                elif e == 2:
                    self.arm.drop3()  # 放下物品
                    e += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            else:
                rospy.logerr("task error: navigation to the drop_place fails")
                if e == 0:
                    self.arm.drop1()  # 放下物品
                    e += 1
                elif e == 1:
                    self.arm.drop2()  # 放下物品
                    e += 1
                elif e == 2:
                    self.arm.drop3()  # 放下物品
                    e += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            b = b - 1
            c = 1
            if b != 0:
                print("返回san点")
                ret = self.robot.goto_local('san')
                rospy.sleep(1)
                if self.stop_flag: return  
#若san点没C和D，则顺路抓B     
        if b==0 and c == 0 and san != 1:   #为达成上述循环与此if执行一个的效果，定义C标志位，来实现此效果。
            # self.robot.step_back()  # 后退
            # if self.stop_flag: return
            item_type = self.arm.detectorB() 
            if len(cube_list2) == 1:
                san = 1
            print("若san点没C和D，则顺路抓B")
            if item_type[0][0] == object_list[0][0]:
                print("========扫描中，准备抓取===== ")
                self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
                print("========向后退一点===== ")
                self.robot.step_back()  # 后退
                if self.stop_flag: return
                self.arm.arm_grasp_ready()
                print("========前往放置区===== ")
                ret = self.robot.goto_local('six')
                if self.stop_flag: return
                ret = self.robot.goto_local("Collection_B") # 根据抓到的物品类型，导航到对应的放置区
                #rospy.sleep(2) # 停稳
                if self.stop_flag: return 
                # self.robot.step_go1()
                if self.stop_flag: return        
                if ret:
                    if d == 0:
                        self.arm.drop1()  # 放下物品
                        d += 1
                    elif d == 1:
                        self.arm.drop2()  # 放下物品
                        d += 1
                    elif d == 2:
                        self.arm.drop3()  # 放下物品
                        d += 1
                    else:
                        self.arm.drop()  # 放下物品
                    self.robot.step_back1()  # 后退
                    self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                    if self.stop_flag: return
                else:
                    rospy.logerr("task error: navigation to the drop_place fails")
                    if d == 0:
                        self.arm.drop1()  # 放下物品
                        d += 1
                    elif d == 1:
                        self.arm.drop2()  # 放下物品
                        d += 1
                    elif d == 2:
                        self.arm.drop3()  # 放下物品
                        d += 1
                    else:
                        self.arm.drop()  # 放下物品
                    self.robot.step_back1()  # 后退
                    self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            else:
                print("去si点")              
                ret = self.robot.goto_local('six')
                if self.stop_flag: return          
        print("去wu点")              
        ret = self.robot.goto_local('wu')
        if self.stop_flag: return 
#在wu点抓C  
        b = 0
        a = 0
        c = 0
        if ret: 
            item_type = self.arm.detectorC()
            print("在wu点抓C")
            b = a
        while b > 0 and wu != 1:
            # self.robot.step_back()  # 后退
            # if self.stop_flag: return
            if b == 3:
                print("3个C")
                item_type = self.arm.detectorC()
            elif b == 2:
                print("2个C")
                item_type = self.arm.detectorC()
            elif b == 1:
                print("1个C")
                item_type = self.arm.detectorC()
            if len(cube_list2) == 1:
                wu = 1
            print("========扫描中，准备抓取===== ")
            self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
            print("========向后退一点===== ")
            self.robot.step_back()  # 后退
            if self.stop_flag: return
            self.arm.arm_grasp_ready()
            print("========前往放置区===== ")
            ret = self.robot.goto_local("Collection_C") # 根据抓到的物品类型，导航到对应的放置区
            #rospy.sleep(2) # 停稳
            if self.stop_flag: return 
            # self.robot.step_go1()
            if self.stop_flag: return        
            if ret:
                if e == 0:
                    self.arm.drop1()  # 放下物品
                    e += 1
                elif e == 1:
                    self.arm.drop2()  # 放下物品
                    e += 1
                elif e == 2:
                    self.arm.drop3()  # 放下物品
                    e += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            else:
                rospy.logerr("task error: navigation to the drop_place fails")
                if e == 0:
                    self.arm.drop1()  # 放下物品
                    e += 1
                elif e == 1:
                    self.arm.drop2()  # 放下物品
                    e += 1
                elif e == 2:
                    self.arm.drop3()  # 放下物品
                    e += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            b = b - 1
            if wu != 1:
                print("返回wu点")
                ret = self.robot.goto_local('wu')
                if self.stop_flag: return 
            else :
                print("返回wu点")
                ret = self.robot.goto_local('liuy')
                if self.stop_flag: return 
                print("返回wu点")
                ret = self.robot.goto_local('qi')
                if self.stop_flag: return 
#在wu点抓B     
        b = 0
        a = 0 
        c = 0
        if ret and wu != 1: 
            item_type = self.arm.detectorB()
            print("在wu点抓B")
            b = a
        while b > 0 and wu != 1:
            # self.robot.step_back()  # 后退
            # if self.stop_flag: return
            if b == 3:
                print("3个B")
                item_type = self.arm.detectorB()
            elif b == 2:
                print("2个B")
                item_type = self.arm.detectorB()
            elif b == 1:
                print("1个B")
                item_type = self.arm.detectorB()
            if len(cube_list2) == 1:
                wu = 1
            print("========扫描中，准备抓取===== ")
            self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
            print("========向后退一点===== ")
            self.robot.step_back()  # 后退
            if self.stop_flag: return
            self.arm.arm_grasp_ready()
            print("========前往放置区===== ")
            ret = self.robot.goto_local("Collection_B") # 根据抓到的物品类型，导航到对应的放置区
            #rospy.sleep(2) # 停稳
            if self.stop_flag: return 
            # self.robot.step_go1()
            if self.stop_flag: return        
            if ret:
                if d == 0:
                    self.arm.drop1()  # 放下物品
                    d += 1
                elif d == 1:
                    self.arm.drop2()  # 放下物品
                    d += 1
                elif d == 2:
                    self.arm.drop3()  # 放下物品
                    d += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready() # 移动机械臂到其他地方
                if self.stop_flag: return
            else:
                rospy.logerr("task error: navigation to the drop_place fails")
                if d == 0:
                    self.arm.drop1()  # 放下物品
                    d += 1
                elif d == 1:
                    self.arm.drop2()  # 放下物品
                    d += 1
                elif d == 2:
                    self.arm.drop3()  # 放下物品
                    d += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            b = b - 1
            c = 1
            if b != 0:      
                print("返回wu点")
                ret = self.robot.goto_local('wu')
                if self.stop_flag: return 
#在wu点若没有B和C，则顺路去抓D
        if b==0 and c == 0 and wu != 1:      #为达成上述循环与此if执行一个的效果，定义C标志位，来实现此效果。  
            # self.robot.step_back()  # 后退
            # if self.stop_flag: return                          
            item_type = self.arm.detectorD() 
            if len(cube_list2) == 1:
                wu = 1
            print("在wu点若没有B和C，则顺路去抓D")
            if item_type[0][0] == object_list[2][0]:
                print("========扫描中，准备抓取===== ")
                self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
                print("========向后退一点===== ")
                self.robot.step_back()  # 后退
                if self.stop_flag: return
                self.arm.arm_grasp_ready()
                print("========前往放置区===== ")
                ret = self.robot.goto_local('siy')
                if self.stop_flag: return
                ret = self.robot.goto_local("Collection_D") # 根据抓到的物品类型，导航到对应的放置区
                #rospy.sleep(2) # 停稳
                if self.stop_flag: return 
                # self.robot.step_go1()
                if self.stop_flag: return        
                if ret:
                    if f == 0:
                        self.arm.drop1()  # 放下物品
                        f += 1
                    elif f == 1:
                        self.arm.drop2()  # 放下物品
                        f += 1
                    elif f == 2:
                        self.arm.drop3()  # 放下物品
                        f += 1
                    else:
                        self.arm.drop()  # 放下物品
                    self.robot.step_back1()  # 后退
                    self.arm.arm_grasp_ready() # 移动机械臂到其他地方
                    if self.stop_flag: return
                else:
                    rospy.logerr("task error: navigation to the drop_place fails")
                    if f == 0:
                        self.arm.drop1()  # 放下物品
                        f += 1
                    elif f == 1:
                        self.arm.drop2()  # 放下物品
                        f += 1
                    elif f == 2:
                        self.arm.drop3()  # 放下物品
                        f += 1
                    else:
                        self.arm.drop()  # 放下物品
                    self.robot.step_back1()  # 后退
                    self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
                print("返回yi点") 
                ret = self.robot.goto_local('bay')
                if self.stop_flag: return
            else:
                ret = self.robot.goto_local('liuy')
                if self.stop_flag: return           
        print("返回qi点") 
        ret = self.robot.goto_local('qi')
        if self.stop_flag: return   
#在qi点抓B
        b = 0
        a = 0  
        c = 0
        if ret: 
            item_type = self.arm.detectorB()
            print("在qi点抓B")
            b = a
        while b > 0 and qi != 1:
            # self.robot.step_back()  # 后退
            # if self.stop_flag: return
            if b == 3:
                print("3个B")
                item_type = self.arm.detectorB()
            elif b == 2:
                print("2个B")
                item_type = self.arm.detectorB()
            elif b == 1:
                print("1个B")
                item_type = self.arm.detectorB()
            if len(cube_list2) == 1:
                qi = 1
            print("========扫描中，准备抓取===== ")
            self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
            print("========向后退一点===== ")
            self.robot.step_back()  # 后退
            if self.stop_flag: return
            self.arm.arm_grasp_ready()
            print("========前往放置区===== ")
            ret = self.robot.goto_local("Collection_B") # 根据抓到的物品类型，导航到对应的放置区
            #rospy.sleep(2) # 停稳
            if self.stop_flag: return 
            # self.robot.step_go1()
            if self.stop_flag: return        
            if ret:
                if d == 0:
                    self.arm.drop1()  # 放下物品
                    d += 1
                elif d == 1:
                    self.arm.drop2()  # 放下物品
                    d += 1
                elif d == 2:
                    self.arm.drop3()  # 放下物品
                    d += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            else:
                rospy.logerr("task error: navigation to the drop_place fails")
                if d == 0:
                    self.arm.drop1()  # 放下物品
                    d += 1
                elif d == 1:
                    self.arm.drop2()  # 放下物品
                    d += 1
                elif d == 2:
                    self.arm.drop3()  # 放下物品
                    d += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            b = b - 1
            if qi != 1:
                print("返回qi点")
                ret = self.robot.goto_local('qi')
                if self.stop_flag: return  
            else:
                print("返回qi点")
                ret = self.robot.goto_local('bax')     
                if self.stop_flag: return     
                ret = self.robot.goto_local('yi')        
                if self.stop_flag: return  
#在qi点抓D
        b = 0
        a = 0  
        c = 0
        if ret and qi != 1: 
            item_type = self.arm.detectorD()
            print("在qi点抓D")
            b = a
        while b > 0 and qi != 1:
            # self.robot.step_back()  # 后退
            # if self.stop_flag: return
            if b == 3:
                print("3个D")
                item_type = self.arm.detectorD()
            elif b == 2:
                print("2个D")
                item_type = self.arm.detectorD()
            elif b == 1:
                print("1个D")
                item_type = self.arm.detectorD()
            if len(cube_list2) == 1:
                qi = 1
            print("========扫描中，准备抓取===== ")
            self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
            print("========向后退一点===== ")
            self.robot.step_back()  # 后退
            if self.stop_flag: return
            self.arm.arm_grasp_ready()
            print("========前往放置区===== ")
            ret = self.robot.goto_local("bax") # 根据抓到的物品类型，导航到对应的放置区
            #rospy.sleep(2) # 停稳
            if self.stop_flag: return 
            ret = self.robot.goto_local("Collection_D") # 根据抓到的物品类型，导航到对应的放置区
            #rospy.sleep(2) # 停稳
            if self.stop_flag: return 
            # self.robot.step_go1()
            if self.stop_flag: return        
            if ret:
                if f == 0:
                    self.arm.drop1()  # 放下物品
                    f += 1
                elif f == 1:
                    self.arm.drop2()  # 放下物品
                    f += 1
                elif f == 2:
                    self.arm.drop3()  # 放下物品
                    f += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            else:
                rospy.logerr("task error: navigation to the drop_place fails")
                if f == 0:
                    self.arm.drop1()  # 放下物品
                    f += 1
                elif f == 1:
                    self.arm.drop2()  # 放下物品
                    f += 1
                elif f == 2:
                    self.arm.drop3()  # 放下物品
                    f += 1
                else:
                    self.arm.drop()  # 放下物品
                self.robot.step_back1()  # 后退
                self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
            b = b - 1
            if self.stop_flag: return
            c = 1
            if b != 0:
                print("返回qi点")
                ret = self.robot.goto_local('bay')
                if self.stop_flag: return      
                ret = self.robot.goto_local('qi')
                if self.stop_flag: return 
            else:
                ret = self.robot.goto_local('yi')
                if self.stop_flag: return                 
        if b==0 and c == 0 and qi != 1:      #为达成上述循环与此if执行一个的效果，定义C标志位，来实现此效果。     
            # self.robot.step_back()  # 后退
            # if self.stop_flag: return                       
            item_type = self.arm.detectorC() 
            if len(cube_list2) == 1:
                qi = 1
            print("在qi点如果没有B和D，则抓c")
            if item_type[0][0] == object_list[1][0]:
                print("========扫描中，准备抓取===== ")
                self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
                print("========向后退一点===== ")
                self.robot.step_back()  # 后退
                if self.stop_flag: return
                self.arm.arm_grasp_ready()
                print("========前往放置区===== ")
                ret = self.robot.goto_local('liux')
                if self.stop_flag: return
                ret = self.robot.goto_local("Collection_C") # 根据抓到的物品类型，导航到对应的放置区
                #rospy.sleep(2) # 停稳
                if self.stop_flag: return 
                # self.robot.step_go1()
                if self.stop_flag: return        
                if ret:
                    if e == 0:
                        self.arm.drop1()  # 放下物品
                        e += 1
                    elif e == 1:
                        self.arm.drop2()  # 放下物品
                        e += 1
                    elif e == 2:
                        self.arm.drop3()  # 放下物品
                        e += 1
                    else:
                        self.arm.drop()  # 放下物品
                    self.robot.step_back1()  # 后退
                    self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                    if self.stop_flag: return
                else:
                    rospy.logerr("task error: navigation to the drop_place fails")
                    if e == 0:
                        self.arm.drop1()  # 放下物品
                        e += 1
                    elif e == 1:
                        self.arm.drop2()  # 放下物品
                        e += 1
                    elif e == 2:
                        self.arm.drop3()  # 放下物品
                        e += 1
                    else:
                        self.arm.drop()  # 放下物品
                    self.robot.step_back1()  # 后退
                    self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                if self.stop_flag: return
                print("返回er点")
                ret = self.robot.goto_local('erx')
                if self.stop_flag: return 
                ret = self.robot.goto_local('yi')
                if self.stop_flag: return   
            # else:
            #     ret = self.robot.goto_local('ba')
            #     if self.stop_flag: return    
        print("返回yi点")
        # ret = self.robot.goto_local('yi')
        # if self.stop_flag: return     
#第二圈在yi点抓C
#µÚ¶þÈ¦ÔÚyiµã×¥C
        b = 0
        a = 0 
        c = 0    
        item_type = self.arm.detectorend()
        b = len(item_type)
        print("第二圈yi点")
        if b != 0:
            while b > 0:
                # self.robot.step_back()  # 后退
                # if self.stop_flag: return
                item_type = self.arm.detectorend()
                if item_type[0][0] == object_list[0][0]:
                    self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
                    print("========向后退一点===== ")
                    self.robot.step_back()  # 后退
                    if self.stop_flag: return
                    self.arm.arm_grasp_ready()  
                    ret = self.robot.goto_local("bay") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    #rospy.sleep(2) # Í£ÎÈ
                    if self.stop_flag: return              
                    ret = self.robot.goto_local("Collection_B") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    # self.robot.step_go1()
                    if self.stop_flag: return        
                    if ret:
                        if d == 0:
                            self.arm.drop1()  # ·ÅÏÂÎïÆ·
                            d += 1
                        elif d == 1:
                            self.arm.drop2()  # ·ÅÏÂÎïÆ·
                            d += 1
                        elif d == 2:
                            self.arm.drop3()  # ·ÅÏÂÎïÆ·
                            d += 1
                        else:
                            self.arm.drop()  # 放下物品
                        self.robot.step_back1()  # ºóÍË
                        self.arm.arm_grasp_ready()  # ÒÆ¶¯»úÐµ±Ûµ½ÆäËûµØ·½
                        if self.stop_flag: return
                    else:
                        rospy.logerr("task error: navigation to the drop_place fails")
                        if d == 0:
                            self.arm.drop1()  # 放下物品
                            d += 1
                        elif d == 1:
                            self.arm.drop2()  # 放下物品
                            d += 1
                        elif d == 2:
                            self.arm.drop3()  # 放下物品
                            d += 1
                        self.robot.step_back1()  # 后退
                        self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                    if self.stop_flag: return
                    ret = self.robot.goto_local("bax") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    #rospy.sleep(2) # Í£ÎÈ
                    if self.stop_flag: return      
                if item_type[0][0] == object_list[1][0]:
                    self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
                    print("========向后退一点===== ")
                    self.robot.step_back()  # 后退
                    if self.stop_flag: return
                    self.arm.arm_grasp_ready()  
                    ret = self.robot.goto_local("ery") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    #rospy.sleep(2) # Í£ÎÈ
                    if self.stop_flag: return    
                    ret = self.robot.goto_local("Collection_C") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    # self.robot.step_go1()
                    if self.stop_flag: return        
                    if ret:
                        if e == 0:
                            self.arm.drop1()  # ·ÅÏÂÎïÆ·
                            e += 1
                        elif e == 1:
                            self.arm.drop2()  # ·ÅÏÂÎïÆ·
                            e += 1
                        elif e == 2:
                            self.arm.drop3()  # ·ÅÏÂÎïÆ·
                            e += 1
                        else:
                            self.arm.drop()  # 放下物品
                        self.robot.step_back1()  # ºóÍË
                        self.arm.arm_grasp_ready()  # ÒÆ¶¯»úÐµ±Ûµ½ÆäËûµØ·½
                        if self.stop_flag: return
                    else:
                        rospy.logerr("task error: navigation to the drop_place fails")
                        if e == 0:
                            self.arm.drop1()  # 放下物品
                            e += 1
                        elif e == 1:
                            self.arm.drop2()  # 放下物品
                            e += 1
                        elif e == 2:
                            self.arm.drop3()  # 放下物品
                            e += 1
                        else:
                            self.arm.drop()  # 放下物品
                        self.robot.step_back1()  # 后退
                        self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                    if self.stop_flag: return
                    ret = self.robot.goto_local("er") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    #rospy.sleep(2) # Í£ÎÈ
                    if self.stop_flag: return      
                if item_type[0][0] == object_list[2][0]:
                    self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
                    print("========向后退一点===== ")
                    self.robot.step_back()  # 后退
                    if self.stop_flag: return
                    self.arm.arm_grasp_ready()      
                    ret = self.robot.goto_local("Collection_D") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    # self.robot.step_go1()
                    if self.stop_flag: return        
                    if ret:
                        if f == 0:
                            self.arm.drop1()  # ·ÅÏÂÎïÆ·
                            f += 1
                        elif f == 1:
                            self.arm.drop2()  # ·ÅÏÂÎïÆ·
                            f += 1
                        elif f == 2:
                            self.arm.drop3()  # ·ÅÏÂÎïÆ·
                            f += 1
                        else:
                            self.arm.drop()  # 放下物品
                        self.robot.step_back1()  # ºóÍË
                        self.arm.arm_grasp_ready()  # ÒÆ¶¯»úÐµ±Ûµ½ÆäËûµØ·½
                        if self.stop_flag: return    
                    else:
                        rospy.logerr("task error: navigation to the drop_place fails")
                        if f == 0:
                            self.arm.drop1()  # 放下物品
                            f += 1
                        elif f == 1:
                            self.arm.drop2()  # 放下物品
                            f += 1
                        elif f == 2:
                            self.arm.drop3()  # 放下物品
                            f += 1
                        else:
                            self.arm.drop()  # 放下物品
                        self.robot.step_back1()  # 后退
                        self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                    if self.stop_flag: return
                ret = self.robot.goto_local('yi')
                if self.stop_flag: return   
                b = b - 1
        self.robot.step_go2(0.15)
        if self.stop_flag: return   
        b = 0
        a = 0 
        c = 0    
        print("zhongjiandian视觉识别之前")
        item_type = self.arm.detectorend1()
        b = len(item_type)
        print("第二圈zhongjiandian点")
        if b != 0:
            while b>0:      
                # self.robot.step_back()  # 后退
                # if self.stop_flag: return      
                item_type = self.arm.detectorend1()
                if item_type[0][0] == object_list[0][0]:
                    self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
                    print("========向后退一点===== ")
                    self.robot.step_back()  # 后退
                    if self.stop_flag: return
                    self.arm.arm_grasp_ready()  
                    ret = self.robot.goto_local("bay") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    #rospy.sleep(2) # Í£ÎÈ
                    if self.stop_flag: return              
                    ret = self.robot.goto_local("Collection_B") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    # self.robot.step_go1()
                    if self.stop_flag: return        
                    if ret:
                        if d == 0:
                            self.arm.drop1()  # ·ÅÏÂÎïÆ·
                            d += 1
                        elif d == 1:
                            self.arm.drop2()  # ·ÅÏÂÎïÆ·
                            d += 1
                        elif d == 2:
                            self.arm.drop3()  # ·ÅÏÂÎïÆ·
                            d += 1
                        else:
                            self.arm.drop()  # 放下物品
                        self.robot.step_back1()  # ºóÍË
                        self.arm.arm_grasp_ready()  # ÒÆ¶¯»úÐµ±Ûµ½ÆäËûµØ·½
                        if self.stop_flag: return
                    else:
                        rospy.logerr("task error: navigation to the drop_place fails")
                        if d == 0:
                            self.arm.drop1()  # 放下物品
                            d += 1
                        elif d == 1:
                            self.arm.drop2()  # 放下物品
                            d += 1
                        elif d == 2:
                            self.arm.drop3()  # 放下物品
                            d += 1
                        else:
                            self.arm.drop()  # 放下物品
                        self.robot.step_back1()  # 后退
                        self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                    if self.stop_flag: return
                    ret = self.robot.goto_local("bax") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    #rospy.sleep(2) # Í£ÎÈ
                    if self.stop_flag: return      
                if item_type[0][0] == object_list[1][0]:
                    self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
                    print("========向后退一点===== ")
                    self.robot.step_back()  # 后退
                    if self.stop_flag: return
                    self.arm.arm_grasp_ready()  
                    ret = self.robot.goto_local("ery") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    #rospy.sleep(2) # Í£ÎÈ
                    if self.stop_flag: return    
                    ret = self.robot.goto_local("Collection_C") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    # self.robot.step_go1()
                    if self.stop_flag: return        
                    if ret:
                        if e == 0:
                            self.arm.drop1()  # ·ÅÏÂÎïÆ·
                            e += 1
                        elif e == 1:
                            self.arm.drop2()  # ·ÅÏÂÎïÆ·
                            e += 1
                        elif e == 2:
                            self.arm.drop3()  # ·ÅÏÂÎïÆ·
                            e += 1
                        else:
                            self.arm.drop()  # 放下物品
                        self.robot.step_back1()  # ºóÍË
                        self.arm.arm_grasp_ready()  # ÒÆ¶¯»úÐµ±Ûµ½ÆäËûµØ·½
                        if self.stop_flag: return
                    else:
                        rospy.logerr("task error: navigation to the drop_place fails")
                        if e == 0:
                            self.arm.drop1()  # 放下物品
                            e += 1
                        elif e == 1:
                            self.arm.drop2()  # 放下物品
                            e += 1
                        elif e == 2:
                            self.arm.drop3()  # 放下物品
                            e += 1
                        else:
                            self.arm.drop()  # 放下物品
                        self.robot.step_back1()  # 后退
                        self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                    if self.stop_flag: return
                    ret = self.robot.goto_local("erx") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    #rospy.sleep(2) # Í£ÎÈ
                    if self.stop_flag: return      
                if item_type[0][0] == object_list[2][0]:
                    self.arm.grasp(item_type)  # 抓取物品并返回抓取物品的类型
                    print("========向后退一点===== ")
                    self.robot.step_back()  # 后退
                    if self.stop_flag: return
                    self.arm.arm_grasp_ready()      
                    ret = self.robot.goto_local("Collection_D") # ¸ù¾Ý×¥µ½µÄÎïÆ·ÀàÐÍ£¬µ¼º½µ½¶ÔÓ¦µÄ·ÅÖÃÇø
                    # self.robot.step_go1()
                    if self.stop_flag: return        
                    if ret:
                        if f == 0:
                            self.arm.drop1()  # ·ÅÏÂÎïÆ·
                            f += 1
                        elif f == 1:
                            self.arm.drop2()  # ·ÅÏÂÎïÆ·
                            f += 1
                        elif f == 2:
                            self.arm.drop3()  # ·ÅÏÂÎïÆ·
                        else:
                            self.arm.drop()  # 放下物品
                        self.robot.step_back1()  # ºóÍË
                        self.arm.arm_grasp_ready()  # ÒÆ¶¯»úÐµ±Ûµ½ÆäËûµØ·½
                        if self.stop_flag: return    
                    else:
                        rospy.logerr("task error: navigation to the drop_place fails")
                        if f == 0:
                            self.arm.drop1()  # 放下物品
                            f += 1
                        elif f == 1:
                            self.arm.drop2()  # 放下物品
                            f += 1
                        elif f == 2:
                            self.arm.drop3()  # 放下物品
                            f += 1
                        else:
                            self.arm.drop()  # 放下物品
                        self.robot.step_back1()  # 后退
                        self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
                    if self.stop_flag: return 
                ret = self.robot.goto_local("yi")
                if self.stop_flag: return   
                self.robot.step_go2(0.15)
                if self.stop_flag: return   
                b = b - 1

        exit()


        self.arm.arm_home()
        # act.goto_local("sp")

        rospy.logwarn("***** task finished *****")
        rospy.logwarn("if you want to run task again. ")
        rospy.logwarn("Re-send a message to hm_task_cmd topic. ")
        rospy.logwarn("Or press Ctrl+C to exit the program")

    def task_cmd_cb(self,flag):
        if flag :
            if not self.task_run_th.is_alive():
                self.stop_flag = False
                self.task_run_th = threading.Thread(target=self.run_task, args=())
                self.task_run_th.start()
                rospy.loginfo("start task!!!")
            else:
                rospy.logwarn("waiting for thread exit...")
                self.stop_flag = True
                self.task_run_th.join()
                rospy.logwarn("thread exit success")

    def grasp_cb(self, msg):
        if not self.task_run_th.is_alive():
            if msg.data == "1":
                self.arm.grasp()
            elif msg.data == "0":
                self.arm.drop()
                self.arm.arm_grasp_ready()
            else:
                rospy.logwarn("grasp msg error")


if __name__ == '__main__':
    try:
        rospy.Subscriber("/objects", Detection2DArray, cube_list_callback,queue_size=1)
        AutoAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Mark_move finished.")
