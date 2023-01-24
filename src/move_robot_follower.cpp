#include "ros/ros.h"
#include <iostream>
#include <std_msgs/String.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"


#include "move_base_msgs/MoveBaseGoal.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_datatypes.h>

// Code von https://answers.ros.org/question/210987/sending-a-sequence-of-goals-to-move_base/ übernommen und angepasst

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveRobotFollower
{
private:
    ros::Subscriber sub_cam;
    ros::Subscriber sub_odom;
    std::vector<std::vector <double>> std_goals;
    move_base_msgs::MoveBaseGoal current_goal;
    int goal_index;
    double pose_x, pose_y, pose_yaw; // x in [m], y in [m], yaw in rad
public:
    MoveBaseClient ac;
    MoveRobotFollower(/* args */);
    ~MoveRobotFollower();
    void standardPath();
    void followRobot(int section);
    void callbackCamera(const std_msgs::String::ConstPtr& msg);
    void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg);
};

MoveRobotFollower::MoveRobotFollower(): ac("move_base", true)
{
    ros::NodeHandle nh("~");
    sub_cam = nh.subscribe("sub_cam_x_pos", 10, &MoveRobotFollower::callbackCamera, this);
    sub_odom = nh.subscribe("sub_odom_follower", 10, &MoveRobotFollower::callbackOdom, this);

    current_goal.target_pose.header.frame_id = "map";
    goal_index = 0;

    // read the goals defined in the yaml file (maps folder)
    std_goals.resize(10);
    
    ros::param::get("/robot1/move_robot_follower/goal1", std_goals[0]);
    ros::param::get("/robot1/move_robot_follower/goal2", std_goals[1]);
    ros::param::get("/robot1/move_robot_follower/goal3", std_goals[2]);
    ros::param::get("/robot1/move_robot_follower/goal4", std_goals[3]);
    ros::param::get("/robot1/move_robot_follower/goal5", std_goals[4]);
    ros::param::get("/robot1/move_robot_follower/goal6", std_goals[5]);
    ros::param::get("/robot1/move_robot_follower/goal7", std_goals[6]);
    ros::param::get("/robot1/move_robot_follower/goal8", std_goals[7]);
    ros::param::get("/robot1/move_robot_follower/goal9", std_goals[8]);
    ros::param::get("/robot1/move_robot_follower/goal10", std_goals[9]);

    //std::cout << "\n\n\ngoals: x:" << goals[0][0] << " y:" << goals[0][1] << std::endl;
    //std::cout << "goals: x:" << goals[1][0] << " y:" << goals[1][1] << std::endl;
    //std::cout << "goals: x:" << goals[2][0] << " y:" << goals[2][1] << "\n" << std::endl;

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
}

MoveRobotFollower::~MoveRobotFollower()
{
}

void MoveRobotFollower::callbackOdom(const nav_msgs::Odometry::ConstPtr& odom_data)
{
    //std::cout << "================ODOM=================" << std::endl;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_data->pose.pose.orientation, quat);
    
    pose_x = odom_data->pose.pose.position.x;
    pose_y = odom_data->pose.pose.position.y;
    pose_yaw = tf::getYaw(quat);
}

void MoveRobotFollower::callbackCamera(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "================CAMERA============================" << std::endl;
    std::cout << "Received message: " << msg->data.c_str() << std::endl;

    int section = std::stoi(msg->data.c_str());
    // TImer wär noch nice, sofern einmal dedektiert, dass das Ziel nicht überschrieben wird zu standardPath wir auch wenn er den ROboter verloren hat
    switch (section)
    {
    case 0:
        standardPath();
        break;
    case 1:
        followRobot(section);
        break;
    case 2:
        followRobot(section);
        break;
    case 3:
        followRobot(section);
        break;
    case 4:
        followRobot(section);
        break;
    case 5:
        followRobot(section);
        break;
    default:
        break;
    }
}

void MoveRobotFollower::followRobot(int section)
{
    ROS_INFO("\n--------------FOLLOW PATH ----------------\n");
    
    // Odometry Daten aus den Klassen-Variablen nehmen und damit die Ausrichtung vom Roboter auf Weltkoordinaten-System umrechnen
    // Anhand der Zahl section das Ziel pose_x, pose_y Werte berechnen und Rotation pose_yaw, so dass es immer nach vorne schaut
    
    //current_goal.target_pose.header.stamp = ros::Time::now();
    //current_goal.target_pose.pose.position.x = goals[goal_index][0];
    //current_goal.target_pose.pose.position.y = goals[goal_index][1];
    //current_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(new_yaw);

    //ac.sendGoal(current_goal);
}

void MoveRobotFollower::standardPath()
{
    ROS_INFO("\n____________Standard PATH ______________\n");

    double yaw = std_goals[goal_index][2]; // the orientation around the z-axis in radians
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);

    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.pose.position.x = std_goals[goal_index][0];
    current_goal.target_pose.pose.position.y = std_goals[goal_index][1];
    current_goal.target_pose.pose.orientation = quat;

    ac.sendGoal(current_goal);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goals");
    MoveRobotFollower mba;

    ros::Rate rate(10); // set the rate to 10 hz
    while(ros::ok())
    {
        if(mba.ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Robot reached goal");
            mba.ac.cancelAllGoals();
        }
    }
    ros::spinOnce();
    rate.sleep();
    return 0;
}