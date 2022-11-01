#include "ros/ros.h"    //The ROS main header
#include <math.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


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
        cv::Mat m_cameraImage;

        //----------------------Methoden----------------------//
        Camera();
        void callbackCamera(const sensor_msgs::ImageConstPtr& cameraImage);

    private:
        ros::Publisher tf_pub;    ///< Publisher für die erkannte Kugelposition
        ros::Subscriber sub_camera;      ///< Subscriber für die Kamera
        cv::Mat imageCb(const sensor_msgs::ImageConstPtr& msg);
};

cv::Mat Camera::imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat image;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      image = cv_ptr->image;
      return image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return image;
    }
  }

void Camera::callbackCamera(const sensor_msgs::ImageConstPtr& cameraImage)
{
    //std::cout << "CALLBACK" << std::endl;
    //std::cout << "Bildbreite: " << cameraImage.width << "\tBildhöhe: " << cameraImage.height << std::endl;
    //output = cameraImage.width.toString();
    if(cameraImage->width != 0){
        m_cameraImage = imageCb(cameraImage);
        std::cout << "CALLBACK" << std::endl;
        //std::cout << "cam: " << m_cameraImage << std::endl;
        //cv::namedWindow("test", 0);  
        cv::imshow("test", m_cameraImage);
        cv::waitKey(2000); 

    }
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


