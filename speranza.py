#!/usr/bin/env python3
"""macchina_a_stati.py  –  Task‑2 (pick 1 ➜ place 4)

Script completo (versione 25‑05‑2025):
    • salva le pose ArUco (1‑4) su file
    • porta il braccio in pose intermedia, poi start‑pose
    • esegue pick‑and‑place 1 → 4 con IK semplificata

Funziona con i topic pubblicati da `testa_e_aruco.py` e i controller joint‐trajectory
standard di TIAGo.  Ready to run.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from pathlib import Path
import yaml
import numpy as np
from math import cos, sin
from typing import Dict, List, Optional

# ───────── CONFIG ───────────────────────────────────────────────────────────
ARM_JOINT_NAMES = [
    'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
    'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
]
INTERMEDIATE_ARM_Q = [
    3.836728901962516e-04, -1.633239063343339e-04, -9.037018213753356e-06,
   -6.145563957549172e-05, 4.409014973383307e-05, 1.9643255648595925e-03,
    4.167305736686444e-04,
]
START_ARM_Q = [0.07, 0.10, -3.10, 1.36, 2.05, 0.01, -0.05]
START_TORSO = 0.35
GRIPPER_JOINT_NAMES = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
GRIPPER_OPEN, GRIPPER_CLOSE = [0.04, 0.04], [0.0, 0.0]
SAFETY_Z_OFFSET, PICK_OFFSET, PLACE_OFFSET = 0.05, 0.06, 0.06
DEFAULT_DURATION, GRIPPER_DURATION = 4.0, 2.0
POSE_FILE = Path.home() / 'aruco_poses_task2.yaml'

class Step:
    def __init__(self, kind: str, target, duration: float):
        self.kind, self.target, self.duration = kind, target, duration

# ───────── MAIN NODE ───────────────────────────────────────────────────────
class MacchinaStati(Node):
    def __init__(self):
        super().__init__('macchina_a_stati_task2')
        # action client
        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       '/arm_controller/follow_joint_trajectory')
        self.torso_client = ActionClient(self, FollowJointTrajectory,
                                         '/torso_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory,
                                           '/gripper_controller/follow_joint_trajectory')
        for c in (self.arm_client, self.torso_client, self.gripper_client):
            c.wait_for_server()
        # marker pose buffer
        self.marker_pose: Dict[int, PoseStamped] = {}
        for mid in [1, 2, 3, 4]:
            self.create_subscription(PoseStamped, f'/aruco_pose_base_{mid}',
                                     lambda msg, m=mid: self.marker_pose.__setitem__(m, msg), 10)
        # state vars
        self.state = -2; self.busy = False; self.marker_saved = False
        self.steps: List[Step] = []; self.current_step = 0; self.last_q = START_ARM_Q.copy()
        self.timer = self.create_timer(0.1, self.loop)
        self.move_to_intermediate_pose()

    # ───────── core loop ──────────────────────────────────────────────────
    def loop(self):
        if self.busy or self.state == 99:
            return
        if not self.marker_saved and len(self.marker_pose) == 4:
            self.save_marker_poses(); self.marker_saved = True
        if self.state == -1:
            self.state = 0; self.get_logger().info('⌛ waiting for markers 1&4')
        if self.state == 0 and 1 in self.marker_pose and 4 in self.marker_pose:
            self.plan_sequence(); self.state = 1
        if self.state == 1:
            if self.current_step < len(self.steps):
                st = self.steps[self.current_step]
                if st.kind == 'pose':
                    self.send_arm_goal(st.target, st.duration)
                else:
                    self.send_gripper_goal(st.target == 'open', st.duration)
                self.busy = True; self.current_step += 1
            else:
                self.get_logger().info('🏁 done'); self.state = 99

    # ───────── transitions ───────────────────────────────────────────────
    def move_to_intermediate_pose(self):
        self.send_arm_joint_goal(INTERMEDIATE_ARM_Q, DEFAULT_DURATION, self._intermediate_done)
        self.busy = True
    def _intermediate_done(self):
        self.busy = False; self.move_to_start_pose()
    def move_to_start_pose(self):
        self.state = -1
        traj_torso = JointTrajectory(joint_names=['torso_lift_joint'])
        traj_torso.points.append(JointTrajectoryPoint(positions=[START_TORSO],
                                     time_from_start=Duration(sec=int(DEFAULT_DURATION))))
        self.torso_client.send_goal_async(FollowJointTrajectory.Goal(trajectory=traj_torso))
        self.send_arm_joint_goal(START_ARM_Q, DEFAULT_DURATION, self._start_pose_done)
        self.busy = True
    def _start_pose_done(self):
        self.busy = False

    # ───────── planning ──────────────────────────────────────────────────
    def plan_sequence(self):
        ps1, ps4 = self.marker_pose[1], self.marker_pose[4]
        q_down = [0.0, 1.0, 0.0, 0.0]
        def off(ps, dz):
            p=PoseStamped(); p.header=ps.header; p.pose.position.x=ps.pose.position.x; p.pose.position.y=ps.pose.position.y; p.pose.position.z=ps.pose.position.z+dz; p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=q_down; return p
        src_h, src_l = off(ps1, SAFETY_Z_OFFSET), off(ps1, -PICK_OFFSET)
        dst_h, dst_l = off(ps4, SAFETY_Z_OFFSET), off(ps4, -PLACE_OFFSET)
        self.steps = [Step('pose', src_h, DEFAULT_DURATION), Step('pose', src_l, DEFAULT_DURATION/2),
                      Step('gripper', 'close', GRIPPER_DURATION), Step('pose', src_h, DEFAULT_DURATION/2),
                      Step('pose', dst_h, DEFAULT_DURATION), Step('pose', dst_l, DEFAULT_DURATION/2),
                      Step('gripper', 'open', GRIPPER_DURATION), Step('pose', dst_h, DEFAULT_DURATION/2)]
        self.current_step = 0

    # ───────── utilities ────────────────────────────────────────────────
    def save_marker_poses(self):
        with POSE_FILE.open('w') as f:
            yaml.dump({m:{'x':p.pose.position.x,'y':p.pose.position.y,'z':p.pose.position.z,
                          'qx':p.pose.orientation.x,'qy':p.pose.orientation.y,'qz':p.pose.orientation.z,'qw':p.pose.orientation.w} for m,p in self.marker_pose.items()}, f)
        self.get_logger().info('💾 pose saved in %s', POSE_FILE)

    # ───────── IK & FK (dummy) ───────────────────────────────────────────
    def dummy_fk(self, q: np.ndarray):
        l1,l2,l3 = 0.3,0.3,0.2
        x = l1*cos(q[0])+l2*cos(q[0]+q[1])+l3*cos(q[0]+q[1]+q[2])
        y = l1*sin(q[0])+l2*sin(q[0]+q[1])+l3*sin(q[0]+q[1]+q[2])
        z = 0.8+0.1*q[2]
        return np.array([x,y,z])
    def inverse_kinematics(self, q0: List[float], target: Pose, it=200, eps=1e-3):
        q = np.array(q0,float)
        for _ in range(it):
            err = np.array([target.position.x, target.position.y, target.position.z]) - self.dummy_fk(q)
            if np.linalg.norm(err)<eps: return True, q.tolist()
            J = np.zeros((3,len(q))); J[:3,:3]=np.identity(3)
            q += 0.2*J.T@err
        return False, q0

    # ───────── action helpers ───────────────────────────────────────────
    def _on_action_finished(self, cb):
        self.busy = False; cb and cb()
    def send_arm_joint_goal(self,q:List[float],dur:float,cb=None):
        traj=JointTrajectory(joint_names=ARM_JOINT_NAMES)
        traj.points.append(JointTrajectoryPoint(positions=q,time_from_start=Duration(sec=int(dur))))
        self.arm_client.send_goal_async(FollowJointTrajectory.Goal(trajectory=traj)).add_done_callback(
            lambda ft:self._arm_handle(ft.result(),cb))
    def _arm_handle(self,gh,cb):
        if not gh.accepted:
            self.get_logger().error('arm goal rejected'); self.busy=False; return
        gh.get_result_async().add_done_callback(lambda _:self._on_action_finished(cb))
    def send_gripper_goal(self, open_:bool,dur:float):
        traj=JointTrajectory(joint_names=GRIPPER_JOINT_NAMES)
        traj.points.append(JointTrajectoryPoint(positions=GRIPPER_OPEN if open_ else GRIPPER_CLOSE,
                                               time_from_start=Duration(sec=int(dur))))
        self.gripper_client.send_goal_async(FollowJointTrajectory.Goal(trajectory=traj)).add_done_callback(
            lambda _:self._on_action_finished(None))
    def send_arm_goal(self, pose:PoseStamped,dur:float):
        ok,q=self.inverse_kinematics(self.last_q,pose.pose)
        if not ok:
            self.get_logger().warn('IK failed'); return
        self.last_q=q; self.send_arm_joint_goal(q,dur)

# ───────── main ────────────────────────────────────────────────────────────
def main():
    rclpy.init(); node=MacchinaStati(); rclpy.spin(node)
if __name__=='__main__':
    main()
