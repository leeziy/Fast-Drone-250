#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

/********** WCET **********/
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <atomic> 

namespace ego_planner {
  extern std::atomic<double> waypoint_wcet;
  extern std::atomic<double> odometry_wcet;
  extern std::atomic<double> execFSM_wcet;
  extern std::atomic<double> checkCollision_wcet;
}

extern std::atomic<double> vis_wcet;
extern std::atomic<double> updateOccupancy_wcet;
extern std::atomic<double> depthOdom_wcet;

void SigHandle(int sig)
{
    if (sig == SIGUSR1)
    {
        waypoint_wcet.store(0.0);
        odometry_wcet.store(0.0);
        execFSM_wcet.store(0.0);
        checkCollision_wcet.store(0.0);
        ::vis_wcet.store(0.0);
        ::updateOccupancy_wcet.store(0.0);
        ::depthOdom_wcet.store(0.0);
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

  signal(SIGUSR1, SigHandle);
  pthread_setname_np(pthread_self(), "ego_main");
  ros::waitForShutdown();
  ROS_INFO("Stopping EgoPlanner...");
  
  double waypoint_worst = waypoint_wcet.load();
  printf("=== waypoint WCET: %.0f us ===\n", waypoint_worst * 1000000);
  
  double odometry_worst = odometry_wcet.load();
  printf("=== odometry WCET: %.0f us ===\n", odometry_worst * 1000000);
  
  double execFSM_worst = execFSM_wcet.load();
  printf("=== execFSM WCET: %.0f us ===\n", execFSM_worst * 1000000);
  
  double checkCollision_worst = checkCollision_wcet.load();
  printf("=== checkCollision WCET: %.0f us ===\n", checkCollision_worst * 1000000);
    
  double vis_worst = ::vis_wcet.load();
  printf("=== vis WCET: %.0f us ===\n", vis_worst * 1000000);
  
  double updateOccupancy_worst = ::updateOccupancy_wcet.load();
  printf("=== updateOccupancy WCET: %.0f us ===\n", updateOccupancy_worst * 1000000);
  
  double depthOdom_worst = ::depthOdom_wcet.load();
  printf("=== depthOdom WCET: %.0f us ===\n", depthOdom_worst * 1000000);
  
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
