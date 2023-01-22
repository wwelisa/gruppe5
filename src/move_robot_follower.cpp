#include "ros/ros.h"
#include <iostream>

#include "move_base_msgs/MoveBaseGoal.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_datatypes.h>

// Code von https://answers.ros.org/question/210987/sending-a-sequence-of-goals-to-move_base/ übernommen und angepasst

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goals");

    // read the goals defined in the yaml file (maps folder)
    std::vector<std::vector <double>> goals;
    goals.resize(10);
    ros::param::get("/robot1/move_robot_follower/goal1", goals[0]);
    ros::param::get("/robot1/move_robot_follower/goal2", goals[1]);
    ros::param::get("/robot1/move_robot_follower/goal3", goals[2]);
    ros::param::get("/robot1/move_robot_follower/goal4", goals[3]);
    ros::param::get("/robot1/move_robot_follower/goal5", goals[4]);
    ros::param::get("/robot1/move_robot_follower/goal6", goals[5]);
    ros::param::get("/robot1/move_robot_follower/goal7", goals[6]);
    ros::param::get("/robot1/move_robot_follower/goal8", goals[7]);
    ros::param::get("/robot1/move_robot_follower/goal9", goals[8]);
    ros::param::get("/robot1/move_robot_follower/goal10", goals[9]);

    //std::cout << "\n\n\ngoals: x:" << goals[0][0] << " y:" << goals[0][1] << std::endl;
    //std::cout << "goals: x:" << goals[1][0] << " y:" << goals[1][1] << std::endl;
    //std::cout << "goals: x:" << goals[2][0] << " y:" << goals[2][1] << "\n" << std::endl;
    
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
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
            double yaw = goals[i][2]; // the orientation around the z-axis in radians
            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);
            goal.target_pose.pose.orientation = quat;
            
            ac.sendGoal(goal);
            //std::cout << "\nNew goal sent " << i << ":\tx: " << goals[i][0] << "\ty: " << goals[i][1] << "\ttheta: " << goals[i][2] << std::endl;
            
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
