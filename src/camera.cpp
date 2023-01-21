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

using namespace cv;
using namespace std;


cv::Mat Segment(Mat img){
    //create the result image the same dimensions as the original
    int img_row = img.rows;
    int img_col = img.cols;
    Mat resultImg(img_row, img_col, CV_32F, cv::Scalar(100));

    //iterate through the image
    for(int i = 0; i < img_row; i++){
        for(int j = 0; j < img_col; j++){       
            resultImg.at<float>(i, j) = 2* (int)img.at<Vec3b>(i, j)[0] - (int)img.at<Vec3b>(i, j)[1] - (int)img.at<Vec3b>(i, j)[2];   //2B-R-G   img = BGR 
        }

    }

    //converte the result image to 8U 
    resultImg.convertTo(resultImg, CV_8U);

    return resultImg;
}

class Camera
{
    public:
        //--------------------- Variablen -------------------//
        bool first_measurement;     ///< template Variable

        std_msgs::String output;  ///< Pose mit Kovarianz, welche gepublisht wird und in RVIZ dargestellt
        cv::Mat m_cameraImage;

        //----------------------Methoden----------------------//
        Camera();
        void callbackCamera(const sensor_msgs::ImageConstPtr& cameraImage);

    private:
        ros::Publisher tf_pub;    ///< Publisher für die erkannte Kugelposition
        ros::Subscriber sub_camera;      ///< Subscriber für die Kamera
        cv::Mat imageCb(const sensor_msgs::ImageConstPtr& msg);
        void ImageProcessing();
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

Camera::Camera()
{
    ros::NodeHandle node("~");
    
    sub_camera = node.subscribe("sub_topic", 1000, &Camera::callbackCamera, this); //Camera-image
    
    tf_pub = node.advertise<std_msgs::String>("pub_cam_tf", 1);

}

void Camera::callbackCamera(const sensor_msgs::ImageConstPtr& cameraImage)
{
    //std::cout << "CALLBACK" << std::endl;
    //std::cout << "Bildbreite: " << cameraImage.width << "\tBildhöhe: " << cameraImage.height << std::endl;
    //output = cameraImage.width.toString();
    if(cameraImage->width != 0){
        m_cameraImage = imageCb(cameraImage);
        ImageProcessing();
    }
    output.data = "text";
    tf_pub.publish(output);
}

void Camera::ImageProcessing(){

  cv::Mat img = m_cameraImage.clone();
  cv::Mat img_segment = Segment(img);
  cv::Mat img_edges, img_result;

  cv::Canny(img_segment, img_edges, 0, 10, 5);

  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(img_edges.clone(), contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  //vector<RotatedRect> minRect( contours.size() );
  vector<RotatedRect> minEllipse( contours.size() );

  for( size_t i = 0; i < contours.size(); i++ )
  {
      //minRect[i] = minAreaRect( contours[i] );
      if( contours[i].size() > 5 )
      {
          minEllipse[i] = fitEllipse( contours[i] );
      }
  }

  for( size_t i = 0; i < contours.size(); i++ ){

    ellipse( img, minEllipse[i], CV_RGB(((int)rand() % 255), ((int)rand() % 255), ((int)rand() % 255)), 2 );  
    cv::Point2f center = minEllipse[i].center;
    cout << "center x: " << center.x << " y: " << center.y << " i: " <<  i << " size: " << minEllipse[i].size << "\n\n\n\n";
    cv::putText(img, // target image
                to_string(i), // text
                center, // top-left position
                cv::FONT_HERSHEY_DUPLEX,
                1.0,
                CV_RGB(((int)rand() % 255), ((int)rand() % 255), ((int)rand() % 255)), // font color
                1);
  }

  

  /*
  cv::putText(img, // target image
                "hello", // text
                cv::Point(20,20), // top-left position
                cv::FONT_HERSHEY_DUPLEX,
                1.0,
                CV_RGB(((int)rand() % 255), ((int)rand() % 255), ((int)rand() % 255)), // font color
                1);
  */

  cv::imshow("test", img);
  cv::waitKey(20); 
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf");
    
    Camera cam;
    std::cout << "\n\n\n\n\n\n\n\n\nMAIN" << std::endl;
    
    ros::spin();
}


