#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Livox LiDAR模式控制测试脚本

该脚本用于测试livox_ros_driver新增的激光雷达工作模式控制功能。
支持通过ROS服务控制单个或所有激光雷达的工作模式（Normal/PowerSaving/Standby）。
"""

import rospy
import sys
import time
from std_srvs.srv import Empty
from livox_ros_driver.srv import LidarModeControl, LidarModeControlRequest
from livox_ros_driver.msg import LidarStatus
from std_msgs.msg import String

class LidarControlTester:
    def __init__(self):
        rospy.init_node('lidar_control_tester', anonymous=True)
        
        # 服务客户端
        self.mode_control_client = rospy.ServiceProxy(
            'livox/lidar_mode_control', 
            LidarModeControl
        )
        
        # 状态订阅者
        self.status_sub = rospy.Subscriber(
            'livox/status', 
            LidarStatus, 
            self.status_callback
        )
        
        # 存储接收到的状态信息
        self.lidar_status = {}
        
        # 等待服务可用
        rospy.loginfo("等待激光雷达模式控制服务...")
        self.mode_control_client.wait_for_service(timeout=10.0)
        rospy.loginfo("激光雷达模式控制服务已就绪")
        
    def status_callback(self, msg):
        """处理激光雷达状态消息"""
        self.lidar_status[msg.broadcast_code] = {
            'handle': msg.handle,
            'state': msg.state,
            'mode': msg.mode,
            'is_connected': msg.is_connected,
            'is_sampling': msg.is_sampling,
            'error_code': msg.error_code
        }
        rospy.loginfo(f"收到激光雷达 {msg.broadcast_code} 状态: "
                     f"状态={msg.state}, 模式={msg.mode}, "
                     f"连接={msg.is_connected}, 采样={msg.is_sampling}")
    
    def print_lidar_status(self):
        """打印所有激光雷达状态"""
        if not self.lidar_status:
            rospy.logwarn("未收到任何激光雷达状态信息")
            return
            
        rospy.loginfo("=== 当前激光雷达状态 ===")
        for broadcast_code, status in self.lidar_status.items():
            state_str = self.get_state_string(status['state'])
            mode_str = self.get_mode_string(status['mode'])
            rospy.loginfo(f"激光雷达 {broadcast_code}:")
            rospy.loginfo(f"  Handle: {status['handle']}")
            rospy.loginfo(f"  状态: {state_str}")
            rospy.loginfo(f"  模式: {mode_str}")
            rospy.loginfo(f"  连接: {'是' if status['is_connected'] else '否'}")
            rospy.loginfo(f"  采样: {'是' if status['is_sampling'] else '否'}")
            if status['error_code'] != 0:
                rospy.logwarn(f"  错误码: 0x{status['error_code']:08X}")
            rospy.loginfo("")
    
    def get_state_string(self, state):
        """将状态码转换为字符串"""
        states = {
            0: "初始化",
            1: "正常",
            2: "省电",
            3: "待机",
            4: "错误",
            5: "未知"
        }
        return states.get(state, f"未知({state})")
    
    def get_mode_string(self, mode):
        """将模式码转换为字符串"""
        modes = {
            1: "正常模式",
            2: "省电模式",
            3: "待机模式"
        }
        return modes.get(mode, f"未知({mode})")
    
    def set_lidar_mode(self, broadcast_code, mode):
        """设置指定激光雷达的模式"""
        try:
            request = LidarModeControlRequest()
            request.broadcast_code = broadcast_code
            request.mode = mode
            
            rospy.loginfo(f"正在设置激光雷达 {broadcast_code} 为模式 {mode}...")
            response = self.mode_control_client(request)
            
            if response.success:
                rospy.loginfo(f"成功设置激光雷达 {broadcast_code} 模式: {response.message}")
            else:
                rospy.logerr(f"设置激光雷达 {broadcast_code} 模式失败: {response.message}")
            
            return response.success
            
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False
    
    def set_all_lidar_mode(self, mode):
        """设置所有激光雷达的模式"""
        try:
            request = LidarModeControlRequest()
            request.broadcast_code = ""  # 空字符串表示所有激光雷达
            request.mode = mode
            
            rospy.loginfo(f"正在设置所有激光雷达为模式 {mode}...")
            response = self.mode_control_client(request)
            
            if response.success:
                rospy.loginfo(f"成功设置所有激光雷达模式: {response.message}")
            else:
                rospy.logerr(f"设置所有激光雷达模式失败: {response.message}")
            
            return response.success
            
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False
    
    def run_interactive_test(self):
        """运行交互式测试"""
        rospy.loginfo("=== Livox激光雷达模式控制测试 ===")
        rospy.loginfo("等待接收激光雷达状态信息...")
        
        # 等待接收一些状态信息
        time.sleep(3)
        
        while not rospy.is_shutdown():
            print("\n" + "="*50)
            print("Livox激光雷达模式控制测试菜单")
            print("="*50)
            print("1. 显示当前激光雷达状态")
            print("2. 设置指定激光雷达为正常模式")
            print("3. 设置指定激光雷达为省电模式")
            print("4. 设置指定激光雷达为待机模式")
            print("5. 设置所有激光雷达为正常模式")
            print("6. 设置所有激光雷达为省电模式")
            print("7. 设置所有激光雷达为待机模式")
            print("0. 退出")
            print("-"*50)
            
            try:
                choice = input("请选择操作 (0-7): ").strip()
                
                if choice == '0':
                    rospy.loginfo("退出测试")
                    break
                elif choice == '1':
                    self.print_lidar_status()
                elif choice in ['2', '3', '4']:
                    if not self.lidar_status:
                        rospy.logwarn("未检测到连接的激光雷达")
                        continue
                    
                    print("可用的激光雷达:")
                    for i, broadcast_code in enumerate(self.lidar_status.keys()):
                        print(f"  {i+1}. {broadcast_code}")
                    
                    try:
                        lidar_choice = input("请选择激光雷达编号: ").strip()
                        lidar_index = int(lidar_choice) - 1
                        broadcast_codes = list(self.lidar_status.keys())
                        
                        if 0 <= lidar_index < len(broadcast_codes):
                            broadcast_code = broadcast_codes[lidar_index]
                            mode = int(choice)  # 2->正常, 3->省电, 4->待机
                            self.set_lidar_mode(broadcast_code, mode)
                        else:
                            rospy.logwarn("无效的激光雷达编号")
                    except ValueError:
                        rospy.logwarn("请输入有效的数字")
                elif choice in ['5', '6', '7']:
                    mode = int(choice) - 3  # 5->2(正常), 6->3(省电), 7->4(待机)
                    self.set_all_lidar_mode(mode)
                else:
                    rospy.logwarn("无效的选择")
                
                # 等待一段时间让状态更新
                time.sleep(2)
                
            except KeyboardInterrupt:
                rospy.loginfo("用户中断，退出测试")
                break
            except Exception as e:
                rospy.logerr(f"发生错误: {e}")
    
    def run_automated_test(self):
        """运行自动化测试"""
        rospy.loginfo("=== 开始自动化测试 ===")
        
        # 等待接收状态信息
        rospy.loginfo("等待接收激光雷达状态信息...")
        time.sleep(5)
        
        if not self.lidar_status:
            rospy.logwarn("未检测到连接的激光雷达，测试终止")
            return
        
        # 显示初始状态
        rospy.loginfo("初始状态:")
        self.print_lidar_status()
        
        # 测试设置所有激光雷达为省电模式
        rospy.loginfo("\n测试1: 设置所有激光雷达为省电模式")
        if self.set_all_lidar_mode(2):  # 省电模式
            time.sleep(5)
            rospy.loginfo("设置后的状态:")
            self.print_lidar_status()
        
        # 测试设置所有激光雷达为正常模式
        rospy.loginfo("\n测试2: 设置所有激光雷达为正常模式")
        if self.set_all_lidar_mode(1):  # 正常模式
            time.sleep(5)
            rospy.loginfo("设置后的状态:")
            self.print_lidar_status()
        
        # 测试设置单个激光雷达
        if len(self.lidar_status) > 0:
            broadcast_code = list(self.lidar_status.keys())[0]
            rospy.loginfo(f"\n测试3: 设置激光雷达 {broadcast_code} 为待机模式")
            if self.set_lidar_mode(broadcast_code, 3):  # 待机模式
                time.sleep(5)
                rospy.loginfo("设置后的状态:")
                self.print_lidar_status()
            
            rospy.loginfo(f"\n测试4: 恢复激光雷达 {broadcast_code} 为正常模式")
            if self.set_lidar_mode(broadcast_code, 1):  # 正常模式
                time.sleep(5)
                rospy.loginfo("最终状态:")
                self.print_lidar_status()
        
        rospy.loginfo("=== 自动化测试完成 ===")

def main():
    try:
        tester = LidarControlTester()
        
        if len(sys.argv) > 1 and sys.argv[1] == 'auto':
            # 自动化测试模式
            tester.run_automated_test()
        else:
            # 交互式测试模式
            tester.run_interactive_test()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS中断")
    except Exception as e:
        rospy.logerr(f"测试过程中发生错误: {e}")

if __name__ == '__main__':
    main() 