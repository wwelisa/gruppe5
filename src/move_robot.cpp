#include "ros/ros.h"            //The ROS main header
#include <iostream>

#include "move_base_msgs/MoveBaseGoal.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Code von https://answers.ros.org/question/210987/sending-a-sequence-of-goals-to-move_base/ übernommen und angepasst

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * Aus einem yaml-file werden die Zielposen ausgelesen. Diese werden dann nacheinander angefahren. Dabei wird das aktuelle Ziel an den navigation stack gepublisht.
 * @brief Die Main
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goals");

    std::vector<std::vector <double>> goals;
    goals.resize(4);
    ros::param::get("/move_rob/goal1", goals[0]);
    ros::param::get("/move_rob/goal2", goals[1]);
    ros::param::get("/move_rob/goal3", goals[2]);
    ros::param::get("/move_rob/goal4", goals[3]);
    
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // fährt alle Ziele nacheinander an und beginnt dann wieder mit dem ersten
    while(1)
    {
        for(int i = 0; i < goals.size(); i++ ) {
            ros::Duration(1).sleep();   // benötigt sleep, ansonsten wird das nächste Ziel übersprungen
            goal.target_pose.pose.position.x = goals[i][0];
            goal.target_pose.pose.position.y = goals[i][1];
            goal.target_pose.pose.orientation.w = goals[i][2];
            //std::cout << "new goal " << i << ":\tx: " << goals[i][0] << "\ty: " << goals[i][1] << "\ttheta: " << goals[i][2] << std::endl;
            
            ac.sendGoal(goal);
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Robot reached goal %d", i);
            }else{
                ROS_INFO("The base failed to reach the goal");
            }
        }
    }
    return 0;
}