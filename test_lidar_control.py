#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
from livox_ros_driver.srv import LidarModeControl, LidarModeControlRequest
from livox_ros_driver.msg import LidarStatus
from std_msgs.msg import String

class LidarController:
    def __init__(self):
        rospy.init_node('lidar_controller_test', anonymous=True)
        
        # 等待服务可用
        rospy.wait_for_service('livox/lidar_mode_control')
        self.mode_control_service = rospy.ServiceProxy('livox/lidar_mode_control', LidarModeControl)
        
        # 订阅状态话题
        self.status_sub = rospy.Subscriber('livox/status', LidarStatus, self.status_callback)
        
        # 存储激光雷达状态
        self.lidar_status = {}
        
        print("LiDAR Controller initialized. Available commands:")
        print("1. Set all LiDARs to normal mode")
        print("2. Set all LiDARs to power-saving mode")
        print("3. Set all LiDARs to standby mode")
        print("4. Set specific LiDAR mode (need broadcast code)")
        print("5. Show connected LiDARs")
        print("6. Exit")
    
    def status_callback(self, msg):
        """处理激光雷达状态消息"""
        self.lidar_status[msg.broadcast_code] = {
            'handle': msg.handle,
            'state': msg.state,
            'mode': msg.mode,
            'connected': msg.is_connected,
            'sampling': msg.is_sampling,
            'error_code': msg.error_code
        }
    
    def set_all_lidar_mode(self, mode):
        """设置所有激光雷达模式"""
        try:
            req = LidarModeControlRequest()
            req.broadcast_code = ""  # 空字符串表示所有激光雷达
            req.mode = mode
            
            response = self.mode_control_service(req)
            print(f"Set all LiDARs to mode {mode}: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False
    
    def set_specific_lidar_mode(self, broadcast_code, mode):
        """设置指定激光雷达模式"""
        try:
            req = LidarModeControlRequest()
            req.broadcast_code = broadcast_code
            req.mode = mode
            
            response = self.mode_control_service(req)
            print(f"Set LiDAR {broadcast_code} to mode {mode}: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False
    
    def show_connected_lidars(self):
        """显示已连接的激光雷达"""
        print("\nConnected LiDARs:")
        print("-" * 80)
        print(f"{'Broadcast Code':<20} {'Handle':<6} {'State':<12} {'Mode':<12} {'Connected':<10} {'Sampling':<10}")
        print("-" * 80)
        
        for broadcast_code, status in self.lidar_status.items():
            state_names = {0: 'Init', 1: 'Normal', 2: 'PowerSaving', 3: 'StandBy', 4: 'Error', 5: 'Unknown'}
            mode_names = {1: 'Normal', 2: 'PowerSaving', 3: 'Standby'}
            
            state_name = state_names.get(status['state'], 'Unknown')
            mode_name = mode_names.get(status['mode'], 'Unknown')
            connected = "Yes" if status['connected'] else "No"
            sampling = "Yes" if status['sampling'] else "No"
            
            print(f"{broadcast_code:<20} {status['handle']:<6} {state_name:<12} {mode_name:<12} {connected:<10} {sampling:<10}")
        
        if not self.lidar_status:
            print("No LiDARs connected or status not received yet.")
        print()
    
    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            try:
                choice = input("Enter your choice (1-6): ").strip()
                
                if choice == '1':
                    self.set_all_lidar_mode(1)  # Normal mode
                elif choice == '2':
                    self.set_all_lidar_mode(2)  # Power-saving mode
                elif choice == '3':
                    self.set_all_lidar_mode(3)  # Standby mode
                elif choice == '4':
                    broadcast_code = input("Enter broadcast code: ").strip()
                    if broadcast_code:
                        mode = int(input("Enter mode (1:Normal, 2:PowerSaving, 3:Standby): ").strip())
                        self.set_specific_lidar_mode(broadcast_code, mode)
                    else:
                        print("Invalid broadcast code")
                elif choice == '5':
                    self.show_connected_lidars()
                elif choice == '6':
                    print("Exiting...")
                    break
                else:
                    print("Invalid choice. Please enter 1-6.")
                    
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                print(f"Error: {e}")

if __name__ == '__main__':
    try:
        controller = LidarController()
        controller.run()
    except rospy.ROSInterruptException:
        pass 