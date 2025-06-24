#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
offboard_control.py
功能：
 1. 启动后以 50 Hz 在 (0,0,HOVER_ALT) 悬停；
 2. 仅当收到 trajectory_flag == TRAJECTORY_STATUS_READY 的
    quadrotor_msgs/PositionCommand 时，才切入控制模式并更新 setpoint；
 3. 其它状态一律忽略，继续悬停或维持旧指令。
"""

import rospy
from geometry_msgs.msg import Vector3, Point
from mavros_msgs.msg import PositionTarget
from quadrotor_msgs.msg import PositionCommand

HZ = 100.0
HOVER_ALT = 1.5
FRAME = PositionTarget.FRAME_LOCAL_NED   # MAVROS 内部 ENU→NED
TYPE_ALL_FIELDS = 0                      # 不忽略任何字段

class OffboardControl:
    def __init__(self):
        # 订阅 traj_server 发布的 PositionCommand
        self.cmd_sub = rospy.Subscriber(
            '/position_cmd',
            PositionCommand, self.pc_cb, queue_size=1)

        # 发布 setpoint_raw/local
        self.sp_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local',
            PositionTarget, queue_size=10)

        # 状态机：False = 悬停, True = 控制
        self.control_mode = False
        self.latest_pc = None

        rospy.Timer(rospy.Duration(1.0 / HZ), self.timer_cb)
        rospy.loginfo("OffboardControl: started, hovering at (0, 0, %.2f)",
                      HOVER_ALT)

    # ---------- PositionCommand 回调 ----------
    def pc_cb(self, msg):
        """仅接受 READY 状态的 PositionCommand"""
        if msg.trajectory_flag == PositionCommand.TRAJECTORY_STATUS_READY:
            if not self.control_mode:
                rospy.loginfo("OffboardControl: READY command received, "
                              "switching to control mode")
            self.control_mode = True
            self.latest_pc = msg
        else:
            # 其余状态（EMPTY / COMPLETED / ABORT 等）全部忽略
            rospy.loginfo("OffboardControl: ignore PositionCommand with flag %d",
                           msg.trajectory_flag)

    # ---------- 定时发布 ----------
    def timer_cb(self, _):
        sp = PositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.coordinate_frame = FRAME
        sp.type_mask = TYPE_ALL_FIELDS     # 全字段有效

        if self.control_mode and self.latest_pc:
            pc = self.latest_pc
            sp.position    = pc.position
            sp.velocity    = pc.velocity
            sp.acceleration_or_force = pc.acceleration
            sp.yaw         = float(pc.yaw)
            sp.yaw_rate    = float(pc.yaw_dot)
        else:
            # 悬停：固定点 (0,0,HOVER_ALT)，其余字段置零
            sp.position    = Point(0.0, 0.0, HOVER_ALT)
            sp.velocity    = Vector3(0.0, 0.0, 0.0)
            sp.acceleration_or_force = Vector3(0.0, 0.0, 0.0)
            sp.yaw         = 0.0
            sp.yaw_rate    = 0.0

        self.sp_pub.publish(sp)

if __name__ == '__main__':
    rospy.init_node('OffboardControl')
    try:
        OffboardControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
