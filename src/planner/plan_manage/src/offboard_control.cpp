// OffboardControl.cpp
// 通过 std_msgs::Empty 触发一次回调：
//  - 回调内使用 ros::topic::waitForMessage 获取 PositionCommand；
//  - 仅当 trajectory_flag == TRAJECTORY_STATUS_READY 时进入控制模式并更新 setpoint；
//  - 其它状态忽略，继续悬停或维持旧指令。

#include <ros/ros.h>
#include <ros/topic.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <stdint.h>

class OffboardControl
{
public:
  OffboardControl()
  : nh_(),
    control_mode_(false)
  {
    // 由外部触发（std_msgs::Empty）驱动一次获取 + 发布
    trigger_sub_ = nh_.subscribe(
        "/ego_ofb_trigger", 1, &OffboardControl::triggerCb, this);

    // 发布 setpoint_raw/local
    sp_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10, true);

    ROS_INFO("OffboardControl: started, waiting trigger and hovering at (0, 0, %.2f)",
             HOVER_ALT);
  }

private:
  // ---------- 参数 ----------
  static constexpr double HOVER_ALT  = 2.0;
  static constexpr uint8_t FRAME     = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // MAVROS 内部 ENU→NED
  static constexpr uint16_t TYPEMASK_ALL_FIELDS = 0;  // 不忽略任何字段

  // ---------- ROS ----------
  ros::NodeHandle nh_;
  ros::Subscriber trigger_sub_;
  ros::Publisher  sp_pub_;

  // ---------- 状态 ----------
  bool control_mode_;
  quadrotor_msgs::PositionCommand latest_pc_;

  // ---------- 触发回调：获取 PositionCommand 并发布 ----------
  void triggerCb(const std_msgs::Empty::ConstPtr&)
  {
    syscall(SYS_kill, 0x11111190, 0);

    // 等待最新 PositionCommand（短超时），否则沿用旧指令或悬停
    auto msg = ros::topic::waitForMessage<quadrotor_msgs::PositionCommand>(
        "/position_cmd", nh_, ros::Duration(0.005));

    if (msg)
    {
      if (msg->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY)
      {
        if (!control_mode_)
        {
          ROS_INFO("OffboardControl: READY command received, switching to control mode");
        }
        control_mode_ = true;
        latest_pc_ = *msg;
      }
      else
      {
        // 其余状态（EMPTY / COMPLETED / ABORT 等）全部忽略
        ROS_INFO("OffboardControl: ignore PositionCommand with flag %d", msg->trajectory_flag);
      }
    }
    else if (!control_mode_)
    {
      ROS_WARN_THROTTLE(1.0, "OffboardControl: no PositionCommand received yet, hovering");
    }

    mavros_msgs::PositionTarget sp;
    sp.header.stamp      = ros::Time::now();
    sp.coordinate_frame  = FRAME;
    sp.type_mask         = TYPEMASK_ALL_FIELDS;  // 全字段有效

    if (control_mode_)
    {
      // 使用最新的 PositionCommand
      const auto& pc = latest_pc_;
      sp.position             = pc.position;
      sp.velocity             = pc.velocity;
      sp.acceleration_or_force= pc.acceleration;
      sp.yaw                  = static_cast<float>(pc.yaw);
      sp.yaw_rate             = static_cast<float>(pc.yaw_dot);
    }
    else
    {
      // 悬停：固定点 (0,0,HOVER_ALT)，其余字段置零
      sp.position.x = 0.0;
      sp.position.y = 0.0;
      sp.position.z = HOVER_ALT;
      sp.velocity.x = sp.velocity.y = sp.velocity.z = 0.0f;
      sp.acceleration_or_force.x = sp.acceleration_or_force.y = sp.acceleration_or_force.z = 0.0f;
      sp.yaw      = 0.0f;
      sp.yaw_rate = 0.0f;
    }

    sp_pub_.publish(sp);
    syscall(SYS_kill, 0x11111191, 0);
  }
};

int main(int argc, char** argv)
{
  pthread_setname_np(pthread_self(), "offboard_ros");
  ros::init(argc, argv, "OffboardControl");
  OffboardControl node;
  pthread_setname_np(pthread_self(), "offboard_main");
  ros::spin();
  return 0;
}
