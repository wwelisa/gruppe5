// http://wiki.ros.org/roscpp/Overview/Timers
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ros/package.h"

//synch callbacks https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define TIMER 3   // how many sec needed for robot to stand still and then shutdown ros node
#define VEL_TRESHOLD 0.1    // velocity threshold for "standing still"

class StillnessChecker
{
private:
    geometry_msgs::Twist last_velocity1, last_velocity2;
    bool is_still1, is_still2;
    double still_threshold;
    double startTime;
    ros::Timer timer;
    int counter1, counter2;

    //https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
    message_filters::Subscriber<nav_msgs::Odometry> sub1;
    message_filters::Subscriber<nav_msgs::Odometry> sub2;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

public:
    StillnessChecker() : is_still1(false), is_still2(false)
    {
        ros::NodeHandle nh("~");
        sub1.subscribe(nh, "odom_sub1", 1);
        sub2.subscribe(nh, "odom_sub2", 1);
        
        sync.reset(new Sync(MySyncPolicy(10), sub1, sub2));
        sync->registerCallback(boost::bind(&StillnessChecker::odomCallback, this, _1, _2));

        startTime = ros::Time::now().toSec();
        timer = nh.createTimer(ros::Duration(TIMER), &StillnessChecker::timerCallback, this);
        
        still_threshold = VEL_TRESHOLD;  // velocity threshold for "standing still"
        counter1 = 0;
        counter2 = 0;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_1_data, const nav_msgs::Odometry::ConstPtr &odom_2_data)
    {
        geometry_msgs::Twist current_velocity_1 = odom_1_data->twist.twist;
        geometry_msgs::Twist current_velocity_2 = odom_2_data->twist.twist;
        //std::cout << "Current velocity 1: linear " << fabs(current_velocity_1.linear.x) << "\tangular vel: " << fabs(current_velocity_1.angular.z) << std::endl;
        //std::cout << "Current velocity 2: linear " << fabs(current_velocity_2.linear.x) << "\tangular vel: " << fabs(current_velocity_2.angular.z) << std::endl;
        if (fabs(current_velocity_1.linear.x) < still_threshold && fabs(current_velocity_1.angular.z) < still_threshold)
        {
            last_velocity1 = current_velocity_1;
            is_still1 = true;
        }
        else
        {
            is_still1 = false;
        }

        if (fabs(current_velocity_2.linear.x) < still_threshold && fabs(current_velocity_2.angular.z) < still_threshold)
        {
            last_velocity2 = current_velocity_2;
            is_still2 = true;
        }
        else
        {
            is_still2 = false;
        }
    }

    void timerCallback(const ros::TimerEvent &event)
    {
        if(is_still1) counter1++;
        else counter1 = 0;

        if(is_still2) counter2++;
        else counter2 = 0;

        if (counter1 > 3 || counter2 > 3)
        {
            std::cout << "\n\nRobot is standing still!" << std::endl;
            std::cout << "Robot is standing still!" << std::endl;
            std::cout << "Robot is standing still!\n\n" << std::endl;
            
            //writes the time into a file
            std::string path = ros::package::getPath("gruppe5");
            std::string filepath = path + "/Time_Table";
            std::ofstream log(filepath.c_str(), std::ios_base::app | std::ios_base::out);
            double elapsedTime = ros::Time::now().toSec() - startTime;
            std::cout << "Time: " << elapsedTime << std::endl;
            log << elapsedTime << "\t standing still" << "\n";

            ros::shutdown();
        }
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "stillness_checker");
    
    StillnessChecker sc;
    
    ros::spin();
    return 0;
}