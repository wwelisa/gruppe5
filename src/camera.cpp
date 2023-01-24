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


/**
 * Funktion um den blauen Ball aus dem Bild zu segmentieren. Wird jeden frame aufgerufen und dient als input für die 
 * Kanten Erkennung. 
 *
 * @brief Segmentierungs funktion um blaue Objekte zu erkennen.
 * 
 * @param img Der aktulle frame der camera als cv::Mat
 * @return cv::Mat Das segmentierte Bild als maske (nur 0  und 255)
 */

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

    std_msgs::String output;    ///< Zahl von 0-5, 0=Ball nicht im Bild, 1=Ball ganz links, ..., 5=Ball ganz rechts
    cv::Mat m_cameraImage;      ///< Der actuelle Frame der Kamera

    //----------------------Methoden----------------------//
    Camera();
    void callbackCamera(const sensor_msgs::ImageConstPtr& cameraImage);

  private:
    ros::Publisher tf_pub;           ///< Publisher für die erkannte Kugelposition
    ros::Subscriber sub_camera;      ///< Subscriber für die Kamera
    cv::Mat imageCb(const sensor_msgs::ImageConstPtr& msg);     
    void ImageProcessing();         
};

/**
 * Macht aus dem Bild aus der sensor_msgs ein cv::Mat Bild welches zur weiteren Verarbeitung verwendet werden kann.
 *
 * @brief Konvertiert das Bild von einem sensor_msgs::ImageConstPtr& zu einer cv::Mat
 * 
 * @param msg Die aktuelle message mit den Kamerabildern 
 * @return cv::Mat Das Bild der Kamera als cv::Mat
 */
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

/**
 * Constructor wird beim Erstellen des Objects aufgerufen. Subscribed auf das Topic "sub_topic" -> remap im launch file
 * auf das Kamerabild des Fänger Roboters. \n
 * Erstellt den Publisher um die Position des Balls zu publishen.
 *
 * @brief Construct a new Camera:: Camera object, subscribe to "sub_topic" and publish to "pub_cam_tf"
 * 
 */
Camera::Camera()
{
    ros::NodeHandle node("~");
    
    sub_camera = node.subscribe("sub_topic", 1000, &Camera::callbackCamera, this); //Camera-image
    
    tf_pub = node.advertise<std_msgs::String>("pub_cam_tf", 1);

}

/**
 * Callback des Camerasubscribers, wenn das Bild vorhanden ist (Breite größer als 0) wird es zuerst in ein cv::Mat konvertiert,
 * und dann das Bild verarbeitet. \n
 * Am Ende wird das Ergebnis der Verarbeitung gepublished.
 *
 * @brief Callback des Camerasubscribers
 * 
 * @param cameraImage 
 */
void Camera::callbackCamera(const sensor_msgs::ImageConstPtr& cameraImage)
{
    if(cameraImage->width != 0){
        m_cameraImage = imageCb(cameraImage);
        ImageProcessing();
    }
    tf_pub.publish(output);
}



/**
 * Das Bild (cv::Mat) wird zuerst Segmentiert. Dann werden von dem segmentierten Bild die Kanten mit canny() ermittelt.
 * Aus den Kanten wird mittels findContours() die Konturen gefunden. Diese dienen als Input für minEllipse(), allen Konturen
 * größer als 5 werden wird eine Ellipse "gefitted". Die breiteste Ellipse wird ermittelt und ihr Mittelpunkt dient wird in 
 * einen von 5 Bildabschnitten eingeteilt. Welcher Bildabschnitt ist der Output der Funktion und wird gepublished. \n
 * Wenn kein Ball im Bild erkannt wird wird 0 zurückgegeben.
 * 
 *
 * @brief Bildverarbeitung gibt die umgefähre Position des Balls im Bild zurück.
 */
void Camera::ImageProcessing(){
  //get a copy of the current frame
  cv::Mat img = m_cameraImage.clone();

  //segment by blue
  cv::Mat img_segment = Segment(img);
  cv::Mat img_edges, img_result;

  //get the edges of the segmentd
  cv::Canny(img_segment, img_edges, 0, 10, 5);

  //get the countours of the edges
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(img_edges.clone(), contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  if(contours.size() > 0){
    //use the contours to find the center of the ball
    vector<RotatedRect> minEllipse( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        //if the contour is big enough fit an ellips around it
        if( contours[i].size() > 5 )
        {
            minEllipse[i] = fitEllipse( contours[i] );
        }
    }
    //counter to save the biggest ellipse
    int NumberBiggest = 0;

    for( size_t i = 0; i < contours.size(); i++ ){
      //find the widest ellipse
      if(minEllipse[NumberBiggest].size.width  < minEllipse[i].size.width ){
        NumberBiggest = i;
      }
    }
    cv::Point2f center = minEllipse[NumberBiggest].center;
    ellipse( img, minEllipse[NumberBiggest], CV_RGB(((int)rand() % 255), ((int)rand() % 255), ((int)rand() % 255)), 2 );  
    circle( img, center, 5, Scalar( 0, 0, 255 ), FILLED, LINE_8 );

    float image_fifth = img.cols/5;
    if(center.x <= image_fifth){
      output.data = "1";
    }else if(center.x <= image_fifth*2){
      output.data = "2";
    }else if(center.x <= image_fifth*3){
      output.data = "3";
    }else if(center.x <= image_fifth*4){
      output.data = "4";
    }else if(center.x <= image_fifth*5){
      output.data = "5";
    }

  }else{
    output.data = "0";
  }
  
  cv::imshow("test", img);
  cv::waitKey(20); 
}



/**
 * @brief Erstellt das Camera Objekt
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf");
    
    Camera cam;
    
    ros::spin();
}