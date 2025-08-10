// OffboardControl.cpp
// ROS C++ 实现：
// 1. 启动后以 50 Hz 在 (0,0,HOVER_ALT) 悬停；
// 2. 仅当收到 trajectory_flag == TRAJECTORY_STATUS_READY 的
//    quadrotor_msgs/PositionCommand 时，才切入控制模式并更新 setpoint；
// 3. 其它状态一律忽略，继续悬停或维持旧指令。

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <stdint.h>

/********** WCET **********/
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <atomic> 
std::atomic<double> wcet{0.0};
void SigHandle(int sig)
{
    if (sig == SIGUSR1)
    {
        wcet.store(0.0);
        ROS_WARN("Received SIGUSR1: WCET records cleared!");
        return;
    }
}
/**************************/

class OffboardControl
{
public:
  OffboardControl()
  : nh_(),
    control_mode_(false)
  {
    // 订阅 traj_server 发布的 PositionCommand
    cmd_sub_ = nh_.subscribe(
        "/position_cmd", 1, &OffboardControl::pcCb, this);

    // 发布 setpoint_raw/local
    sp_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);

    // 定时器：50 Hz
    timer_ = nh_.createTimer(ros::Duration(1.0 / HZ),
                             &OffboardControl::timerCb, this);

    ROS_INFO("OffboardControl: started, hovering at (0, 0, %.2f)",
             HOVER_ALT);
  }

private:
  // ---------- 参数 ----------
  static constexpr double HZ         = 50.0;
  static constexpr double HOVER_ALT  = 2.0;
  static constexpr uint8_t FRAME     = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // MAVROS 内部 ENU→NED
  static constexpr uint16_t TYPEMASK_ALL_FIELDS = 0;  // 不忽略任何字段

  // ---------- ROS ----------
  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_;
  ros::Publisher  sp_pub_;
  ros::Timer      timer_;

  // ---------- 状态 ----------
  bool control_mode_;
  quadrotor_msgs::PositionCommand latest_pc_;

  // ---------- PositionCommand 回调 ----------
  void pcCb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
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

  // ---------- 定时发布 ----------
  void timerCb(const ros::TimerEvent&)
  {
    syscall(SYS_kill, 0x11111230, 0);
    auto t0 = std::chrono::steady_clock::now();
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
    auto t1 = std::chrono::steady_clock::now();
    double t_loop = std::chrono::duration<double>(t1 - t0).count();
    double t_loop_old = wcet.load(std::memory_order_relaxed);
    while (t_loop > t_loop_old && !wcet.compare_exchange_weak(t_loop_old, t_loop)) {}
    syscall(SYS_kill, 0x11111231, 0);
  }
};

int main(int argc, char** argv)
{
  pthread_setname_np(pthread_self(), "offboard_ros");
  ros::init(argc, argv, "OffboardControl");
  OffboardControl node;
  pthread_setname_np(pthread_self(), "offboard_main");
  signal(SIGUSR1, SigHandle);
  ros::spin();
  double worst = wcet.load();
  // ROS_WARN("=== offboard_main WCET: %.0f us ===", worst * 1000000);
  printf("=== offboard_main WCET: %.0f us ===\n", worst * 1000000);
  return 0;
}
