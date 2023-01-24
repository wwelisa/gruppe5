#include "ros/ros.h"
#include <iostream>
#include <std_msgs/String.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"


#include "move_base_msgs/MoveBaseGoal.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_datatypes.h>

#define FORWARD_DISTANCE 0.8
#define PI 3.1415

// Code von https://answers.ros.org/question/210987/sending-a-sequence-of-goals-to-move_base/ übernommen und angepasst

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveRobotFollower
{
private:
    ros::Subscriber sub_cam;
    ros::Subscriber sub_odom;
    std::vector<std::vector <double>> std_goals;
    move_base_msgs::MoveBaseGoal current_goal;
    double pose_x, pose_y, pose_yaw; // x in [m], y in [m], yaw in rad
public:
    bool standardActive;
    int check_change;
    int goal_index;
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
    check_change = 100;
    standardActive = true; 

    // read the goals defined in the yaml file (maps folder)
    std_goals.resize(10);
    
    ros::param::get("/robot2/move_robot_follower/goal1", std_goals[0]);
    ros::param::get("/robot2/move_robot_follower/goal2", std_goals[1]);
    ros::param::get("/robot2/move_robot_follower/goal3", std_goals[2]);
    ros::param::get("/robot2/move_robot_follower/goal4", std_goals[3]);
    ros::param::get("/robot2/move_robot_follower/goal5", std_goals[4]);
    ros::param::get("/robot2/move_robot_follower/goal6", std_goals[5]);
    ros::param::get("/robot2/move_robot_follower/goal7", std_goals[6]);
    ros::param::get("/robot2/move_robot_follower/goal8", std_goals[7]);
    ros::param::get("/robot2/move_robot_follower/goal9", std_goals[8]);
    ros::param::get("/robot2/move_robot_follower/goal10", std_goals[9]);

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

/**
 * Die Odometrie wird in diesem Callback aufgerufen. 
 * @brief Speichert die aktuellen Odometrie-Daten in die Klassenvariablen
 * @param odom_data Die aktuellen Odometrie-Daten, die Gazebo liefert. \n Geschwindigkeit [m/s] in X Richtung, sowie die Winkelgeschwindigkeit [rad/s] um Z
 */
void MoveRobotFollower::callbackOdom(const nav_msgs::Odometry::ConstPtr& odom_data)
{
    //std::cout << "================ODOM=================" << std::endl;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_data->pose.pose.orientation, quat);
    
    pose_x = odom_data->pose.pose.position.x;
    pose_y = odom_data->pose.pose.position.y;
    pose_yaw = tf::getYaw(quat);
}

/**
 * Nur wenn eine Veränderung der Kameradaten zustande kommt wird die followRobot() oder standardPath() Funktion aufgerufen
 * @brief Ändert das Bewegungsverhalten des Roboter je nach erhaltener Kamerdetektion
 * @param msg Die Position (Sektor/Abschnitt) der blauen Kugel im Bild als String
 */
void MoveRobotFollower::callbackCamera(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "\n================CAMERA============================" << std::endl;
    std::cout << "\nReceived message: " << msg->data.c_str() << std::endl;

    int section = std::stoi(msg->data.c_str());
    std::cout << "section: " << section << "\tcheck_cahnge: " << check_change << std::endl;

    if(section != check_change)
    {
        // TImer wär noch nice, sofern einmal dedektiert, dass das Ziel nicht überschrieben wird zu standardPath wir auch wenn er den ROboter verloren hat
        if(section != 0) followRobot(section);
        else if(section == 0) standardPath();
        check_change = section;
    }
}

/**
 * Je nach section wird ein Winkel addiert oder subtrahiert, um dann mit der aktuellen POSE des Roboters die Zielpose im WKS zu berechnen und publishen.
 * @brief Bewegungsverhalten des Roboter bei Kamerdetektion des Läufers
 * @param section Beschreibt in welchem Bereich des Bildes sich der Roboter aufhält
 */
void MoveRobotFollower::followRobot(int section)
{
    ROS_INFO("\n--------------FOLLOW PATH ----------------\n");
    standardActive = false;
    
    double x_new, y_new, yaw_new;
    double angle_offset;

    switch (section)
    {
    case 1:
        angle_offset = PI/5;
        break;
    case 2:
        angle_offset = PI/7;
        break;
    case 3:
        angle_offset = 0;
        break;
    case 4:
        angle_offset = -PI/7;
        break;
    case 5:
        angle_offset = -PI/5;
        break;
    default:
        break;
    }

    x_new = pose_x + FORWARD_DISTANCE * cos(pose_yaw + angle_offset);
    y_new = pose_y + FORWARD_DISTANCE * sin(pose_yaw + angle_offset);
    yaw_new = pose_yaw + angle_offset;

    std::cout << "x: " << pose_x << "\ty: " << pose_y << "\tyaw: " << pose_yaw << std::endl;
    std::cout << "x_new: " << x_new << "\ty_new: " << y_new << "\tyaw_new: " << yaw_new << std::endl;

    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.pose.position.x = x_new;
    current_goal.target_pose.pose.position.y = y_new;
    current_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_new);

    ac.sendGoal(current_goal);
}

/**
 * Fährt ein vordefiniertes Ziel an.
 * @brief Bewegungsverhalten, wenn nichts detektiert
 * 
 */
void MoveRobotFollower::standardPath()
{
    ROS_INFO("\n____________Standard PATH ______________\n");
    standardActive = true;
    double yaw = std_goals[goal_index][2]; // the orientation around the z-axis in radians
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);

    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.pose.position.x = std_goals[goal_index][0];
    current_goal.target_pose.pose.position.y = std_goals[goal_index][1];
    current_goal.target_pose.pose.orientation = quat;

    ac.sendGoal(current_goal);
}


/**
 * Objekt der Klasse MoveRobotFollower wir erstellt und es wird der Status des aktuellen Ziels in der Standardroute abgefragt. Wenn dieses erreicht ist, dann wird das nächste ziel aktiviert.
 * @brief Die Main
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goals");
    MoveRobotFollower mba;

    ros::Rate rate(5); // set the rate to 10 hz
    while(ros::ok())
    {
        std::cout << "\n\nMAIN\n" << std::endl;
        //if abfrage nur während standard path activ ist
        if(mba.standardActive && mba.ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Robot reached goal\n\n\n\n");
            
            if(mba.goal_index > 9)
            {
                mba.goal_index = 0;
            } else {
                mba.goal_index++;
                mba.check_change = 100;
            }
        }
    ros::spinOnce();
    rate.sleep();
    }
    return 0;
}


/*! \mainpage Fangen spielen mit 2 Turtlebots
* 
* \section Run
* Um das Programm aufzuführen den Befehl "./startPrograms" im Terminal in dem Ordner scripts ausgeführt werden. Davor muss natürlich das Programm noch mit *catkin_make* kompiliert
* werden. 
*
* \section Anforderungen
* Gazebo-Welt: Turtlebot3 \n
* Roboter: waffle_pi \n
* 
* 
* \section Programmablauf
* Es werden mehrere Nodes parallel ausgeführt. Einerseits die Steuerung der Roboter mittels move_base, anderseits wird die Kamera in einer Node ausgewertet vom Fänger-Roboter. Zudem laufen 2 Nodes die überprüfen, 
* ob die Roboter kollidieren (gefangen) oder ob sich einer der beiden nicht bewegt. Sobald 10 Minuten ohne gefangen errreicht wurden werden alle Programme beendet. Wenn gefangen oder Stilltand, dann wird ebenso alles beendet. Dabei wird immer die vergangene Zeit 
* und der Grund dokumentiert in Time_Table. \n
*
*
* \subsection Skripts
* In dem Bash-Skript startPrograms werden 4 zufällige Zahlen generiert mit denen das Launchfile start.launch aufgerufen wird. Diese zahlen dienen als Spawnkoordinaten für die beiden Roboter.
* Wenn die Node collision oder stillstand beendet wird, dann wird dies im Skript killROS, welches im anderen Skript aufgerufen wird, abgegriffen und beendet alle ROS-Prozesse.
* Wenn dies der Fall ist, dann beginnt es wieder von vorne. Es sind derzeit 20 Durchgänge definiert. 
*
* \section Nodes
* 
* \subsection Kameraauswertung
* Erkennt die Position des Balls im Bild und gibt die ungefähre Position als int von 1 (links) - 5 (rechts) zurück.  Das Ergebnis ist 0 wenn der Ball nicht erkannt wurde.
* 
* \subsection Kollisionabfrage
* Es wird auf die Odoemtrie beider Roboter subscribed und dann mit einem Toleranzwert verglichen, ob sich diese an der gleichen Stelle befinden. Wenn dies der Fall ist, 
* dann wird der die benötigte Zeit in Time_Table mit der Info Kollision geschrieben und die Node killt sich.
*
* \subsection Stillstandsabfrage
* Es wird in einem 5 Sekunden Takt überprüft, ob sich der Roboter bewegt (Odom-Geschwindigkeit in X und um Z). Wenn dies nicht der Fall ist wird ein Zähler erhöht. Sollte dies mehrmals hintereinander passieren, dann 
* wird der die benötigte Zeit in Time_Table mit der Info Stillstand geschrieben und die Node killt sich.
*
* \subsection Bewegungsalgorithmus Fänger
* Sofern keine Detektion der blauen Kugel vom Läufer-Roboter gerade geschiet, wird eine Standardroute abgefahren, um die Kugel zu dedektieren.
* Die Standardroute besteht aus 10 Punkte (goals_follower.yaml). \n
* Wird die Kugel detektiert, dann wird je nach übergebenem Sektor (links, mittig, rechst) ein Winkeloffset zur POSE dazugerechnet und ins WKS umgewandelt, damit die neue POSE dann an move_base gepublisht werden kann.
* Der Roboter fährt dann immer 0.8 m in diese  Richtung und regelt sich. Wenn die Kugel für kurze Zeit verschwindet, dann geht er zur Standardroute zurück.
*
*
* \subsection Bewegungsalgorithmus Läufer
* Aus der goals_runner.yaml Datei werden 10 vordefinierte Punkte ausgelesen. Die werden an den move_base gepublisht und immer aktualisiert, wenn das aktuelle Ziel erreicht wurde. 
* 
*
*
* \section Setup
* Für ein funktionierenden Ablauf müssen diese Schritte durchgeführt werden.
*
* \subsection Model
* Das richtige Model turtlebot3 waffle_pi muss verwendet werden, da dieser eine Kamera besitzt.
* 
* \subsection Pfadplanung
* Damit die Pfadplanung funktioniert muss eventuell das package dwa_local_planner nachinstalliert werden.
* 

* 
* \section Authoren
* Bauer, Keher, Lunkmoss, Salner, Wavruvska
*/