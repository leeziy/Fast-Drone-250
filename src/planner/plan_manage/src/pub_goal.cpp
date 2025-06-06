#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

using namespace std;

ros::Publisher planner_goal_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_goal");
    ros::NodeHandle nh("~");
    
    // 注意查看topic是否对应正确
    planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    double state_desired[3];
                        
    while(ros::ok())
    {
        cout << "Pls input the goal position:"<<endl;
        cout << "desired state: --- x [m] "<<endl;
        cin >> state_desired[0];
        cout << "desired state: --- y [m]"<<endl;
        cin >> state_desired[1];
        cout << "desired state: --- z [m]"<<endl;
        cin >> state_desired[2];

        geometry_msgs::PoseStamped target_point;
        target_point.pose.position.x = state_desired[0];
        target_point.pose.position.y = state_desired[1];
        target_point.pose.position.z = state_desired[2];

        planner_goal_pub.publish(target_point);

        cout << "Goal [X Y Z] : " << state_desired[0] << " [ m ] "<< state_desired[1] <<" [ m ] "<< state_desired[2] <<" [ m ] "<< endl;

        sleep(0.5);
    }

    return 0;
}