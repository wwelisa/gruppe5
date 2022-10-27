#include "ros/ros.h"    //The ROS main header
#include <math.h>
#include <iostream>

// Messages
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"  //http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html


class Camera
{
    public:
        //--------------------- Variablen -------------------//
        bool first_measurement;     ///< template Variable
        
        float vx_odom_now;      ///< Fahrbefehl (Odometrie): Geschwindigkeit [m/s] in X
        float vw_odom_now;      ///< Fahrbefehl (Odometrie): Winkelgeschwindigkeit [rad/s] um Z

        std_msgs::String output;  ///< Pose mit Kovarianz, welche gepublisht wird und in RVIZ dargestellt

        //----------------------Methoden----------------------//
        Camera();
        void callbackCamera(const sensor_msgs::Image& cameraImage);

    private:
        ros::Publisher tf_pub;    ///< Publisher für die erkannte Kugelposition
        ros::Subscriber sub_camera;      ///< Subscriber für die Kamera

};


void Camera::callbackCamera(const sensor_msgs::Image& cameraImage)
{
    std::cout << "CALLBACK" << std::endl;
    std::cout << "Bildbreite: " << cameraImage.width << "\tBildhöhe: " << cameraImage.height << std::endl;
    //output = cameraImage.width.toString();
    output.data = "text";
    tf_pub.publish(output);
}



Camera::Camera()
{
    ros::NodeHandle node("~");
    
    sub_camera = node.subscribe("sub_camera", 1, &Camera::callbackCamera, this); //Camera-image
    
    tf_pub = node.advertise<std_msgs::String>("pub_cam_tf", 1);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf");
    
    Camera cam;
    std::cout << "MAIN" << std::endl;

    ros::spin();
}


