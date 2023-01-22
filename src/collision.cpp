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

#define MAX_TIMER_SEC 600 // time [sec] 10 minutes
#define TOLERANCE 0.35  // distance [m] between robots needed to be considered a successful catch

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_1_data, const nav_msgs::Odometry::ConstPtr &odom_2_data, double startTime)
{
    //ROS_INFO("Both Odoms synced!");
    double x1, y1, x2, y2;
    x1 = odom_1_data->pose.pose.position.x;
    y1 = odom_1_data->pose.pose.position.y;
    x2 = odom_2_data->pose.pose.position.x;
    y2 = odom_2_data->pose.pose.position.y;

    //std::cout << "Odom 1 pose: \tx: " << x1 << "\ty: " << y1 << std::endl;
    //std::cout << "Odom 2 pose: \tx: " << x2 << "\ty: " << y2 << std::endl;

    //checks if robots are touching
    if((x2-TOLERANCE < x1 && x1 < x2+TOLERANCE) && (y2-TOLERANCE < y1 && y1 < y2+TOLERANCE))
    {
        std::cout << "\n\nRobot got chatched!" << std::endl;
        std::cout << "Robot got chatched!" << std::endl;
        std::cout << "Robot got chatched!\n\n" << std::endl;
        
        //writes the time into a file
        std::string path = ros::package::getPath("gruppe5");
        std::string filepath = path + "/Time_Table";
        std::ofstream log(filepath.c_str(), std::ios_base::app | std::ios_base::out);
        double eclapsedTime = ros::Time::now().toSec() - startTime;
        double secs = ros::Time::now().toSec();
        std::cout << "Time: " << eclapsedTime << std::endl;
        log << eclapsedTime << "\n";

        ros::shutdown();
    }
}

// gets Called when timer runs out
void timerCallback(const ros::TimerEvent &event, double startTime)
{
    std::cout << "Programm Closes" << std::endl;
    std::string path = ros::package::getPath("gruppe5");
    std::string filepath = path + "/Time_Table";
    std::ofstream log(filepath.c_str(), std::ios_base::app | std::ios_base::out);
    double eclapsedTime = ros::Time::now().toSec() - startTime;
    double secs = ros::Time::now().toSec();

    std::cout << "Time: " << eclapsedTime << std::endl;
    log << eclapsedTime << "\n";

    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "timer_node");
    ros::NodeHandle nh("~");

    // https://stackoverflow.com/questions/45309584/error-using-boostbind-for-subscribe-callback/45311193#45311193
    double startTime = ros::Time::now().toSec();
    ros::Timer timer = nh.createTimer(ros::Duration(MAX_TIMER_SEC), boost::bind(timerCallback, _1, startTime));
    
    //https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
    message_filters::Subscriber<nav_msgs::Odometry> sub1;
    message_filters::Subscriber<nav_msgs::Odometry> sub2;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    
    sub1.subscribe(nh, "odom1_sub", 1);
    sub2.subscribe(nh, "odom2_sub", 1);
    
    sync.reset(new Sync(MySyncPolicy(10), sub1, sub2));   
    sync->registerCallback(boost::bind(&odomCallback, _1, _2, startTime));

    
    ros::spin();

    return 0;
}