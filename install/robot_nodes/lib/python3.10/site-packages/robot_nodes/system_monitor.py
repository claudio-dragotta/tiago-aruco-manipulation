#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped
from enum import Enum

class State(Enum):
    WAITING_FOR_ARUCO = 1
    MOVE_TO_POSE_1 = 2
    MOVE_TO_POSE_2 = 3
    GRIP_OBJECT = 4
    MOVE_UP_COLA = 5
    MOVE_COLA_TO_FINISH = 6
    RELEASE_OBJECT = 7
    MOVE_TO_HOME = 8
    MOVE_TO_POSE_3 = 9
    MOVE_TO_POSE_4 = 10
    GRIP_OBJECT_2 = 11
    MOVE_UP_PRINGLES = 12
    MOVE_PRINGLES_TO_FINISH = 13
    RELEASE_OBJECT_2 = 14
    MOVE_TO_HOME_2 = 15
    COMPLETED = 16

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        self.markers_found = set()
        self.current_state = State.WAITING_FOR_ARUCO
        self.start_time = self.get_clock().now()
        
        # Subscribers per monitoraggio
        self.create_subscription(Int32, '/marker_discovered', self.marker_callback, 10)
        self.create_subscription(Bool, '/all_markers_found', self.all_markers_callback, 10)
        self.create_subscription(Int32, '/command_topic', self.command_callback, 10)
        self.create_subscription(Int32, '/completed_command_topic', self.completed_callback, 10)
        
        # Timer per reporting periodico
        self.create_timer(30.0, self.system_status_report)
        
        self.get_logger().info("🔍 Sistema di monitoraggio attivo")
        
    def marker_callback(self, msg):
        if msg.data not in self.markers_found:
            self.markers_found.add(msg.data)
            self.get_logger().info(f"📍 Marker {msg.data} scoperto ({len(self.markers_found)}/4)")
            
    def all_markers_callback(self, msg):
        if msg.data:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.get_logger().info(f"🎯 Fase esplorazione completata in {elapsed:.1f}s")
            
    def command_callback(self, msg):
        state = State(msg.data)
        self.get_logger().info(f"▶️  Comando: {state.name}")
        
    def completed_callback(self, msg):
        state = State(msg.data)
        self.current_state = state
        self.get_logger().info(f"✅ Completato: {state.name}")
        
        if state == State.COMPLETED:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.get_logger().info(f"🏆 SISTEMA COMPLETATO SUCCESSFULLY in {elapsed:.1f}s!")
            
    def system_status_report(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.get_logger().info(f"📊 Status: {self.current_state.name} | "
                              f"Markers: {len(self.markers_found)}/4 | "
                              f"Uptime: {elapsed:.1f}s")

def main(args=None):
    rclpy.init(args=args)
    monitor = SystemMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()