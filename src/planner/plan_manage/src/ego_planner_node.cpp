#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

/********** WCET **********/
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <atomic> 

extern std::atomic<double> ego_planner::waypoint_wcet;
extern std::atomic<double> ego_planner::odometry_wcet;
extern std::atomic<double> ego_planner::execFSM_wcet;
extern std::atomic<double> ego_planner::checkCollision_wcet;

extern std::atomic<double> vis_wcet;
extern std::atomic<double> updateOccupancy_wcet;
extern std::atomic<double> depthOdom_wcet;

void SigHandle(int sig)
{
    if (sig == SIGUSR1)
    {
        odometry_wcet.store(0.0);
        checkCollision_wcet.store(0.0);
        execFSM_wcet.store(0.0);
        waypoint_wcet.store(0.0);
        depthOdom_wcet.store(0.0);
        updateOccupancy_wcet.store(0.0);
        vis_wcet.store(0.0);
        odometry_wcet.store(0.0);
        ROS_WARN("Received SIGUSR1: WCET records cleared!");
        return;
    }
}
/**************************/

int main(int argc, char **argv)
{
  pthread_setname_np(pthread_self(), "ego_ros");
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  EGOReplanFSM rebo_replan;

  rebo_replan.init(nh);

  // ros::Duration(1.0).sleep();
  pthread_setname_np(pthread_self(), "ego_spin");
  ros::AsyncSpinner spinner(4);
  signal(SIGUSR1, SigHandle);
  spinner.start();
  pthread_setname_np(pthread_self(), "ego_main");
  ros::waitForShutdown();
  ROS_INFO("Stopping mavros...");
  spinner.stop();

  return 0;
}

// #include <ros/ros.h>
// #include <csignal>
// #include <visualization_msgs/Marker.h>

// #include <plan_manage/ego_replan_fsm.h>

// using namespace ego_planner;

// void SignalHandler(int signal) {
//   if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
//     ros::shutdown();
//   }
// }

// int main(int argc, char **argv) {

//   signal(SIGINT, SignalHandler);
//   signal(SIGTERM,SignalHandler);

//   ros::init(argc, argv, "ego_planner_node", ros::init_options::NoSigintHandler);
//   ros::NodeHandle nh("~");

//   EGOReplanFSM rebo_replan;

//   rebo_replan.init(nh);

//   // ros::Duration(1.0).sleep();
//   ros::AsyncSpinner async_spinner(4);
//   async_spinner.start();
//   ros::waitForShutdown();

//   return 0;
// }
