#include "ros/ros.h"    //The ROS main header
#include <math.h>
#include <iostream>
#include <algorithm>    // std::reverse

// Messages
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Header.h"

//synch callbacks https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
#include <std_msgs/Int32.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <eigen3/Eigen/Dense>   // Matrix-Bibliothek
#include <tf/transform_listener.h> //Quaternionen
#include <ctime>    //rng
#include <cstdlib>  //rng

#define PI2 2*M_PI      ///< zweimal PI
#define RAUSCHEN -0.1   ///< unterer Greznwert für Sensor und Prozessrauschen

class Filter
{
    public:
        bool first_measurement;     ///< Statusvariable damit beim ersten Durchlauf der Zeitschlitz 0.0001 gesetzt wird
        bool laser_status;          ///< Wird True, wenn zum ersten Mal Laserwerte erhalten werden, damit detectLandmark() keine Berechnungen mit nan hat   
        
        //--------------------- Bewegungen -------------------/
        float vx_odom_now;      ///< Fahrbefehl (Odometrie): Geschwindigkeit [m/s] in X
        float vw_odom_now;      ///< Fahrbefehl (Odometrie): Winkelgeschwindigkeit [rad/s] um Z
        float ax_imu;   ///< Messung (IMU): Beschleunigung [m/s^2] in X
        float ay_imu;   ///< Messung (IMU): Beschleunigung [m/s^2] in Y
        float vw_imu;   ///< Messung (IMU): Winkelgeschwindigkeit [rad/s] um Z
        //----------------------------------------------------/
        
        //------------------ POSE --------------------------//
        double x_old;       ///< µ,x (t-1): Position [m] in X zum vorherigen Zeitpunkt
        double y_old;       ///< µ,y (t-1): Position [m] in Y zum vorherigen Zeitpunkt
        double theta_old;   ///< µ,theta (t-1): Winkel [rad] um Z zum vorherigen Zeitpunkt
        
        double x_now_q;         ///< µ_q,x (t): predictete Position [m] in X zum aktuellen Zeitpunkt
        double y_now_q;         ///< µ_q,y (t): predictete Position [m] in Y zum aktuellen Zeitpunkt
        double theta_now_q;     ///< µ_q,theta (t): predicteter Winkel [rad] um Z zum aktuellen Zeitpunkt
        Eigen::Vector3f V_m_q;  ///< Vektor(3,1) mit der predicteten POSE
        
        double x_now;       ///< µ,x (t): correctete Position [m] in X zum aktuellen Zeitpunkt
        double y_now;       ///< µ,y (t): correctete Position [m] in Y zum aktuellen Zeitpunkt
        double theta_now;   ///< µ,theta (t): correcteter Winkel [rad] um Z zum aktuellen Zeitpunkt

        //--------------------------------------------------//
        
        //------------------- Varianz ---------------//
        Eigen::Matrix3f M_var;      ///< correctete Kovarianzmatrix zum aktuellen Zeitpunkt
        Eigen::Matrix3f M_var_old;  ///< correctete Kovarianzmatrix zum vorherigen Zeitpunkt
        Eigen::Matrix3f M_var_q;    ///< predictete Kovarianzmatrix zum aktuellen Zeitpunkt
        //------------------------------------------//
        
        //------------- Zeiten -------------------//
        double t_now;       ///< aktuelle Zeit [s] beim Aufrufen des synchronisierten Callbacks (Odom & IMU)
        double t_old;       ///< vorherige Zeit [s] beim Aufrufen des synchronisierten Callbacks (Odom & IMU)
        double delta_t;     ///< Zeitdifferenz [s] zwischen aktuellem und vorherigem Aufruf des synchronisierten Callbacks (Odom & IMU)
        //----------------------------------------//
        

        std::vector<Eigen::Vector3f> V_mueh_dv;     ///< Array an Vektoren(3,1) "z_dach". Wird für die Correction mittels Landmarken verwendet. 
        std::vector<Eigen::Vector3f> V_muehv;       ///< Array an Vektoren(3,1) "z". Wird für die Correction mittels Landmarken verwendet. 
        std::vector<Eigen::Matrix3f> M_Hv;          ///< Array mit Jacobi-Matrizen. Wird für die Correction mittels Landmarken verwendet.
        std::vector<Eigen::Matrix3f> M_Kv;          ///<  Array mit Kalmangain-Matrizen. Wird für die Correction mittels Landmarken verwendet.
        Eigen::Matrix3f M_I;        ///< Einheitsmatrix
        
        // Landmarken
        std::vector<std::vector <double>> lm;   ///< Vektor mit Landmarken, welche aus einem yaml-file eingelesen werden. Die Landmarke[0] ist der X-Wert, Landmarke[1] ist der Y-Wert und Landmarke[2] ist die Signatur. X un Y befinden sich im Weltkoordinatensystem

        //----------------------- Laser --------------------//
        std::vector<double> laser_val;      ///< Array mit aktuellen Laserwerten [m]
        std::vector<double> laser_x;        ///< Array mit X-Anteil [m] vom Laserpunkt
        std::vector<double> laser_y;        ///< Array mit Y-Anteil [m] vom Laserpunkt
        std::vector<double> laser_angles;   ///< Array mit dem Winkeln [rad] für jeden Laserpunkt. Der Winkel start mit 0 rad von der X-Achse ausgehend und steigt gegen den Uhrzeigersinn
        //--------------------------------------------------//

        // Pose mit Kovarianz
        geometry_msgs::PoseWithCovarianceStamped pose_cov_predict;  ///< Pose mit Kovarianz, welche gepublisht wird und in RVIZ dargestellt

        Filter();
        void predict();
        void correctImu();
        void correctLaser(int lm_index);
        void readLandmarks();
        int detectLandmark();
        float rng(float min);        
        void callback(const nav_msgs::Odometry::ConstPtr &odom_data, const sensor_msgs::Imu::ConstPtr &imu_data);
        void callbackLaser(const sensor_msgs::LaserScan& laser_data);

    private:
        ros::Publisher pose_cov_pub;    ///< Publisher für die Pose mit Kovarianz, um diese in RVIZ darzustellen
        ros::Subscriber sub_laser;      ///< Subscriber für den Laserscan

    // Code übernommen von https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
        message_filters::Subscriber<nav_msgs::Odometry> sub1;
        message_filters::Subscriber<sensor_msgs::Imu> sub2;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync;

};

/**
 * Nach der Prediction wird die dort berechnete Pose noch einmal mittels IMU-Messdaten korregiert. Dieser Schritt wird immer ausgeführt. Je nach Messwerten
 * ändert sich der Kalman-Gain und bestimmt, wie stark die Messung Einfluss auf die Correction haben. Die verwendeten Formeln orientieren sich am Thrun (Seite 51)
 * Zeile 4 bis 7.
 * @brief Die Korrektur der Pose aufgrund von IMU-Messdaten
 * @par Bestimmen der Berechnungsfunktionen
 * Damit die Pose bestimmt werden kann (Zeile 5) muss die Subtarktion von z(t) und h(µ_q(t)) erfolgen. Bei z(t) handelt es sich um einen Vektor, der die Messwerte
 * beinhaltet (Beschleunigung [m/s^2] in X, Beschleunigung [m/s^2] in Y) und Winkelgeschwindigkeit [rad/s] um Z). Nun muss h(µ_q(t)) die predictete Pose auf gleiche
 * Einheiten transformieren. Diese Berechnungen erfolgen bei a_x_q, a_y_q und w_theta_q. Dabei wurden die Formeln a = 2*(s2 - s1) / delta_t^2 und 
 * w = (theta2 - theta1) / delta_t verwendet. Die Ableitungen dieser h(µ_q(t)) Funktion ergibt dann die Jacobi-Matrix H(µ_q(t)) = M_H. Diese wird benötigt um den 
 * Kalman-Gain zu ermitteln, wie im Thrun (Seite 51) Zeile 4 beschrieben ist. Dabei wird noch ein Sensorrauschen M_Q einberechnet, welches mit zufällig generierten
 * Werten definiert wird. Der Kalman-Gain hat somit die Funktion eine Gewichtung durchzuführen, wie stark der Messung vertraut werden soll und in Zeile 5 vom Thrun,
 * dass die Differenz des Messvektor V_z und V_hm_q wieder auf eine Pose umgerechnet wird. Also von Beschleunigungen und Winkelgeschwindigkeit zu Position und
 * Winkel. \n
 * Die Korrektur mit der IMU funktioniert nicht so gut, da die IMU-Werte stark und schnell ihren Wert wechseln und dadurch die berechnete Beschleunigung abweicht.
 * 
 */
void Filter::correctImu() 
{
    //ROS_INFO("Start IMU correction");

    // predicted POSE zu Beschleunigung und Winkelgschwindigkeit umrechnen --> h(µ_q)
    Eigen::Vector3f V_hm_q; //< Vektor h(µ_q)
    float a_x_q = (2*(x_now_q - x_old)) / pow(delta_t,2);
    float a_y_q = (2*(y_now_q - y_old)) / pow(delta_t,2);
    float w_theta_q = (theta_now_q - theta_old) / delta_t;
    V_hm_q(0) = a_x_q;
    V_hm_q(1) = a_y_q;
    V_hm_q(2) = w_theta_q;
    //std::cout << "Correct IMU:" << std::endl;
    //std::cout << "  a_x: " << a_x_q << std::endl;
    //std::cout << "  a_y: " << a_y_q << std::endl;
    //std::cout << "  w: " << w_theta_q << std::endl;
    
//------------------------- part 2--------------
    Eigen::Matrix3f M_H, M_K, M_Ht, M_Q;
    //calc H(t)
    M_H <<  2/pow(delta_t,2),    0,              0,
            0,              2/pow(delta_t,2),    0,
            0,              0,              1/delta_t;
    //std::cout << "M_H:\n" << M_H << std::endl;

    M_Ht = M_H.transpose();
    // Q-Matrix doppelt (auch in prediction)
    M_Q <<  rng(RAUSCHEN), 0, 0,    
            0, rng(RAUSCHEN), 0,
            0, 0, rng(RAUSCHEN);
    //calc Kalman-Gain (Thrun Zeile 12)
    M_K = M_var_q * M_Ht * (M_H * M_var_q * M_Ht + M_Q).inverse();
    M_var = (M_I - M_K * M_H) * M_var_q;
    //std::cout << "M_var:\n" << M_var << std::endl;

    Eigen::Vector3f V_z, V_m_now;
    // Messvektor
    V_z(0) = ax_imu;
    V_z(1) = ay_imu;
    V_z(2) = vw_imu;

    // correctete POSE
    V_m_now = V_m_q + (M_K*0.1*(V_z - V_hm_q));
    x_now = V_m_now(0);
    y_now = V_m_now(1);
    theta_now = V_m_now(2);
    //std::cout << "V_z:\n" << V_z << std::endl;

std::cout << "IMU correct:   " << "\tx: "<< x_now << "\ty: "<< y_now << "\ttheta: "<< theta_now << "\n" << std::endl;
//--------------------------------------------------------------------------------------
    
    // -------- POSE with Covariance ---------
    pose_cov_predict.header.frame_id = "map";
    pose_cov_predict.header.stamp = ros::Time::now();
    pose_cov_predict.pose.pose.position.x = x_now;
    pose_cov_predict.pose.pose.position.y = y_now;
    // set theta
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta_now);
    tf::quaternionTFToMsg(quat, pose_cov_predict.pose.pose.orientation);
    //set covariance
    pose_cov_predict.pose.covariance[0] = M_var(0,0);     //cov pos x
    pose_cov_predict.pose.covariance[7] = M_var(1,1);     //cov pos y
    pose_cov_predict.pose.covariance[35] = M_var(2,2);    //cov rot z
    pose_cov_pub.publish(pose_cov_predict);
    // ---------------------------------

    // Correctete Wert zu vorherigen Werten machen
    M_var_old = M_var;
    x_old = x_now;
    y_old = y_now;
    theta_old = theta_now;
    //ROS_INFO("Ende IMU correction");
}

/**
 * Wenn keine Landmarke dektiert wurde wird die Funktion sofort beendet. Ansonsten wird z_dach berechnet mit der Postion der Landmarke und der Pose. Die Jacobi-Matrix 
 * M_H wird ebenfalls mithilfe der Landmarkenposition errechnet. Abgesehen davon dass sich diese Formeln unterscheidne ist es vom Prinzip gleich wie die EKF-Correction. 
 * Für jeden Sensorwert werden die Matrizen und Vektoren gespeichert und dann aufsummiert. Hierbei entscheidet wieder der Kalmangain wie stark der EInfluss auf
 * das Ergebnis ist. 
 * 
 * @brief Die Korrektur der Pose aufgrund von erkannter Landmarke
 */
void Filter::correctLaser(int lm_index)
{
    if(lm_index == 0) return;    // Wenn keine LAndmarke dektiert wurde wird die Funktion beendet

    ROS_INFO("Start Laser correction");
    M_Hv.clear();
    M_Kv.clear();
    V_muehv.clear();
    V_mueh_dv.clear();

    Eigen::Matrix3f M_H, M_K, M_Ht, M_Q;
    
    // calc z^~ Z-Dach (Thrun Zeile 10)
    double d = pow((lm[lm_index][0] - x_old),2) + pow((lm[lm_index][1] - y_old),2);
    float z_r_d = sqrt(d);
    float z_ang_d = atan2(lm[lm_index][1] - y_old, lm[lm_index][0] - x_old) - theta_old;
    float z_s_d = lm[lm_index][2];
    
    
    // calc H(t) (Thrun Zeile 11)
    M_H <<  (lm[lm_index][0] - x_now_q)/sqrt(d),    (y_old - y_now_q)/sqrt(d),    0,
            (y_now_q - y_old)/d,                    (lm[lm_index][0] - x_now_q)/d,            -1,
            0,                                      0,                                  0;
            
    M_Hv.push_back(M_H);
    //std::cout << "M_H [" << i << "]:\n" << M_H << std::endl;

    M_Ht = M_H.transpose();
    // Q-Matrix doppelt (auch in prediction)
    M_Q <<  rng(RAUSCHEN), 0, 0,
            0, rng(RAUSCHEN), 0,
            0, 0, rng(RAUSCHEN);
    //calc Kalman-Gain (Thrun Zeile 12)
    M_K = M_var_q * M_Ht * (M_H * M_var_q * M_Ht + M_Q).inverse();
    //std::cout << "M_K: "<< M_K << std::endl;
    M_Kv.push_back(M_K);
    
    //-----------für Thrun Zeile 14-------- //
    Eigen::Vector3f V_mueh_d, V_mueh;
    V_mueh_d(0) = z_r_d;
    V_mueh_d(1) = z_ang_d;
    V_mueh_d(2) = z_s_d;
    V_mueh(0) = laser_val.at(0);        //index von landmarke       
    V_mueh(1) = laser_angles.at(0);     //laserwert der landmarke
    V_mueh(2) = 0;

    V_mueh_dv.push_back(V_mueh_d);
    V_muehv.push_back(V_mueh);
    //-----------------------------//

    
    Eigen::Vector3f sum_Kz;
    Eigen::Matrix3f sum_KH;
    // calc sum of matrices and vectors
    std::cout << "M_Kv size: "<< M_Kv.size() << "\tV_muehv size: "<< V_muehv.size() << "\tV_mueh_dv size: "<< V_mueh_dv.size() << std::endl;
    
    //if(M_Kv.size() == 0 || V_muehv.size() == 0 || V_mueh_dv.size() == 0) return;

    for(int i = 0; i < M_Kv.size(); i++)
    {
        sum_Kz += M_Kv.at(i) * (V_muehv.at(i) - V_mueh_dv.at(i));
        sum_KH += M_Kv.at(i) * M_Kv.at(i);
        //std::cout << "M_Hv [" << i << "]:\n" << M_Hv.at(i) << std::endl;
        //std::cout << "M_Kv [" << i << "]:\n" << M_Kv.at(i) << std::endl;
    }
    // mü (t) corrected
        //std::cout << "sum_Kz:\n"<< sum_Kz << std::endl;
        x_now = x_now_q + sum_Kz(0);
        y_now = y_now_q + sum_Kz(1);
        theta_now = theta_now_q + sum_Kz(2);
    // sigma (t) corrected
        M_var = (M_I - sum_KH) * M_var_old;
        //std::cout << "M_var:\n"<< M_var << std::endl;
        
        M_var_old = M_var;
        x_old = x_now;
        y_old = y_now;
        theta_old = theta_now;
        //std::cout << "Laser correct:   " << "x: "<< x_now << "\ty: "<< y_now << "\ttheta: "<< theta_now << std::endl;

    ROS_INFO("finished correction");
}

/**
 * Aufgrund des Fahrbefehls und der vorherigen Pose wird eine neue aktuelle Pose geschätzt. \n
 * @brief Predicted die Pose vom Roboter
 * @par Berechnungen
 * Die Berechnung der predicteten Pose (x,y,theta) erfolgt der Formel g(u(t), µ(t-1)) aus dem Thrun (Seite 169) Zeile 2. Hierbei wird die alte correctete Pose a
 * ddiert mit der einer Berechnung die aus dem Fahrbefehl und der und dem alten Winkel besteht. Hierbei wird als Fahrbefehl (v in x und w um z) die Werte 
 * der Odometrie verwendet, die als Befehl an den Motor interpertiert werden. \n
 * Die Matrix M_Gt ist die Jacobi-Matrix und somit die Ableitung von klein g(u(t), µ(t-1)). Diese ist im Thrun (Seite 169) Zeile 3 zu sehen. Mit dieser und dem 
 * Prozessrauschen M_Rt kann die Varianz predictet werden. Die dafür verwendete Formel ist aus dem Thrun (Seite 169) Zeile 4.
 */
void Filter::predict()
{
    //zeit berechnung ausgeschnitten und callback gepackt

    // predicted POSE
    x_now_q = x_old + ( ((-vx_odom_now/vw_odom_now) * sin(theta_old)) + ((vx_odom_now / vw_odom_now) * sin(theta_old + (vw_odom_now * delta_t)) ));
    y_now_q = y_old + ( ((vx_odom_now/vw_odom_now) * cos(theta_old)) - ((vx_odom_now / vw_odom_now) * cos(theta_old + (vw_odom_now * delta_t)) ));
    theta_now_q = theta_old + (vw_odom_now * delta_t);
    if(theta_now_q < -PI2)
        theta_now_q += PI2;
    else if(theta_now_q > PI2)
        theta_now_q -= PI2;
    
std::cout << "Prediction: " << "\tx: " << x_now_q << "\ty: " << y_now_q << "\ttheta: " << theta_now_q << std::endl;

    Eigen::Matrix3f M_Gt;   //Jacobi-Matrix Bewegung
    M_Gt << 1,0,((vx_odom_now/vw_odom_now) * cos(theta_old)) - ((vx_odom_now / vw_odom_now) * cos(theta_old + (vw_odom_now * delta_t)) ),
            0,1,((vx_odom_now/vw_odom_now) * sin(theta_old)) - ((vx_odom_now / vw_odom_now) * sin(theta_old + (vw_odom_now * delta_t)) ),
            0,0,1;
    //std::cout << "M_Gt:\n" << M_Gt << std::endl;
    
    Eigen::Matrix3f M_Rt;  //Prozessrauschen
    M_Rt << rng(RAUSCHEN), 0, 0,
            0, rng(RAUSCHEN), 0,
            0, 0, rng(RAUSCHEN);
    
    Eigen::Matrix3f M_Gt_t = M_Gt.transpose();  //transponierte Jacobi-Matrix Bewegung

    //M_var_old neu beschreiben mit Sigma
    M_var_q = M_Gt * M_var_old * M_Gt_t + M_Rt;   //Berechnung Varianz-Matrix
    //std::cout << "Varianz:\n" << M_var_q << std::endl;

    V_m_q(0) = x_now_q;
    V_m_q(1) = y_now_q;
    V_m_q(2) = theta_now_q;
}

/**
 * @brief Eine zufälige Zahl wird generiert
 * @param min Der untere Grenzwert der 
 * @return Ein zufällige Zahl zwischen min und |min| wird übergeben
 */
float Filter::rng(float min)
{
    float max = min * -2;
    srand((unsigned)time(NULL));
    return min + static_cast <float> (rand()) / ( static_cast <float> (RAND_MAX/(max)));
}

/**
 * Es werden aus einem yaml-file Landmarken eingelesen und in eine Klassenvariable gespeichert. \n
 * @brief Liest Landmarken ein
 */
void Filter::readLandmarks()
{
    ros::param::get("/ekf/m1", lm[0]);
    //std::cout << "m" << "1" << ": "  << lm[0][0] << "  " << lm[0][1] << "  " << lm[0][2] << std::endl;
    
}

/**
 * Die Odometrie und IMU werden synchronisiert und dieser Callback wird aufgerufen, wenn von beiden Quellen Daten empfangen werden. \n
 * Es wird der Zeitschlitz berechnet, welcher für Berechnungen in der Filter::prediction() und Filter::correctIMU() notwendig ist. Wenn es sich um die erste Messung 
 * handelt, dann wird der Zeitschlitz auf 0.0001 gesetzt, damit keine Division durch 0 in den Berechnungen erfolgt. \n
 * Außerdem werden die notwendigen Daten in Klassenvarieblen gespeichert.
 * @brief Synchronisierte Callback-Funktion des Odometire-Subscribers und des IMU-Subscribers
 * @param odom_data Die aktuellen Odometrie-Daten (Fahrbefehl), die Gazebo liefert. \n Geschwindigkeit [m/s] in X Richtung, sowie die Winkelgeschwindigkeit [rad/s] um Z
 * @param imu_data Die aktuellen IMU-Daten, die Gazebo liefert \n Beschleunigung [m/s^2] in X und Y Richtung, sowie die Winkelgeschwindigkeit [rad/s] um Z
 * @par Aufruf von Funktionen
 * Es wird die prediction() aufgerufen, dannach die Correction mittels IMU und wenn eine Landmarke erkannt wird, dann auch Correction mittels Laserscan
 * @par Aktivieren/Deaktivieren der Lokalisierung mittels Landmarkenerkennung
 * Die letzte Zeile "correctLaser(detectLandmark()); kommentieren, um die Landmarkenerkennung und die Laser-Correction zu deaktivieren
 */
void Filter::callback(const nav_msgs::Odometry::ConstPtr &odom_data, const sensor_msgs::Imu::ConstPtr &imu_data)
{
    //ROS_INFO("Synchronization successful");
    // --------------- speichert die Daten in Klassenvariablen -----------
    if(first_measurement == false)
    {
        t_old = t_now;
        t_now = ros::Time::now().toSec();
        delta_t = t_now - t_old;    //time difference
    }else{
        t_now = ros::Time::now().toSec();
        t_old = t_now;
        delta_t = 0.0001;    //time difference

        first_measurement = false;
    }
    //ROS_INFO(" time now: %f",t_now);
    //ROS_INFO(" time old: %f",t_old);
    //ROS_INFO(" time delta: %f",delta_t);


    //-------------------- Odometrie Winkel ----------------//
    double phi_x, phi_y, phi_z;
    tf::Quaternion quart(odom_data->pose.pose.orientation.x, odom_data->pose.pose.orientation.y, odom_data->pose.pose.orientation.z, odom_data->pose.pose.orientation.w);//Übergabe der Quarterionne
    tf::Matrix3x3  odo_mat(quart);//erstellen der Matrix
    odo_mat.getRPY(phi_x, phi_y, phi_z);//Ermitteln von Phi X, Phi Y und Phi Z
    // ----------------------------------------------------//
    vx_odom_now = odom_data->twist.twist.linear.x;
    vw_odom_now = odom_data->twist.twist.angular.z;
std::cout << "Odom pose: \tx: " << odom_data->pose.pose.position.x << "\ty: " << odom_data->pose.pose.position.y << "\ttheta: " << phi_z << std::endl;
    
    //std::cout << "Odom:" << std::endl;
    //std::cout << "  v: " << vx_odom_now << std::endl;
    //std::cout << "  w: " << vw_odom_now << std::endl;
    ax_imu = imu_data->linear_acceleration.x;
    ay_imu = imu_data->linear_acceleration.y;
    vw_imu = imu_data->angular_velocity.z;

    //std::cout << "Imu:" << std::endl;
    //std::cout << "  a_x: " << ax_imu << std::endl;
    //std::cout << "  a_y: " << ay_imu << std::endl;
    //std::cout << "  w: " << vw_imu << std::endl;


    predict();  //rechnet aufgrund des Fahrbefehls die POSE
    correctImu();   //rechnet aufgrund der Imu-Messung die POSE
    correctLaser(detectLandmark());    //wenn landmarke erkannt wird, dann wird mit dieser korregiert
}

/**
 * Das Laserscan-KS und das Roboter-KS liegen übereinander mit einer Verschiebung in Z-Richtung. \n
 * Die Laserwerte, welche größer als 2.7 m sind werden herausgefiltert. Dies verhindert, dass mit inf Werten gerechnet wird und reduziert die Rechenzeiten
 * für die Landmarkenerkennung. Außerdem wurde der Landmarkenerkennungsalgorithmus für diesen Abstandsbereich optimiert bzw. ausgelegt. \n
 * Neben dem Abspeichern der Daten in eine Klassenvariable wird der
 * @brief Callback-Funktion des Laserscanner-Subscribers
 * @param laser_data Die aktuellen Laserscan-Daten, die Gazebo liefert.
 */
void Filter::callbackLaser(const sensor_msgs::LaserScan& laser_data)
{
    laser_status = true;
    for(int i = 0; i < laser_data.ranges.size(); i++)
    {
        if(laser_data.ranges[i] >= 2.7)
        {
            laser_val.at(i) = 333;
            laser_x.at(i) = 333;
            laser_y.at(i) = 333;
            continue;   //filter out laser values above 2.7m so that the landmark detection has less data to process
        }
        laser_val.at(i) = laser_data.ranges[i];
        laser_x.at(i) = laser_data.ranges[i] * cos(laser_angles.at(i)); // x-anteil vom laserpunkt
        laser_y.at(i) = laser_data.ranges[i] * sin(laser_angles.at(i)); // y-anteil vom laserpunkt
        //ROS_INFO("Laser Distanz [m] %d: %f", i, laser_data.ranges[i]);
        //ROS_INFO("Winkel Start %f: ", laser_data.angle_min*(180/M_PI));
        //ROS_INFO("Winkel Inkrement [deg]: %f", laser_data.angle_increment * 180.0 / M_PI);
    }
}

/**
 * Bei der Landmarke handelt es sich um ein Eck, sprich zwei Wände treffen aufeinander mit einem bestimmten Winkel. Um diese Landmarke mit den Laserpunkten 
 * eine Form zu erkennen wurde entschieden, dass es notwendig ist zu erkennen ob die Punkte auf einer Linie liegen. Dies wurde mit der Berechnung der Steigung 
 * durchgeführt.
 * @brief Wertet die Laserscandaten aus und ermittelt ob die Landmarke dektiert wird
 * @return returniert die ID der Landmarke (0 wenn keine dektiert)
 * @par Berechnung der Steigung
 * In der callbackLaser Funktion werden bereits Laserwerte, die größer als 2.7 m sind rausgefiltert. Außerdem werden die X- und Y-Anteile für die LAserpunkte dort 
 * bereits ermittelt. Nun wird die Steigung zwischen 2 Laserpunkten berechnet mittels k = y2-y1 / x2-x1. Da die Sensorwerte nebeneinander nur einen kleinen 
 * Abstand haben und doch deutlich rauschen kann die Steigung stark varrieren. Aus diesem Grund werden 8 Punkte ausgelassen. Als Beispiel: Die Steigung zwischen Punkt 
 * 3 und Punkt 11 wird berechnet. Im nächsten Durchlauf der for-Schleife ist es Punkt 4 und Punkt 12. Dadurch variieren die Steigungen weniger, wenn die Punkte auf 
 * einer Linie liegen. Und es ist davon auszugehen, dass mit dem Abstand (<2.7 m) Roboter zu Landmarke genügend Punkte auf der Linie sind. Die Parameter sind 
 * eventuell für die Anwendung anzupassen.
 * @par Linie generieren
 * Damit man sagen kann das es eine Linie gibt werden die Steigungen miteinander verglichen. Sofern diese gleich sind mit einbezogerner Toleranz, werden
 * die Indexe der Laserpunkte abgespeichert. Sobald 5 Steigungen zur Referenzsteigung unterschiedlich sind werden die bisher gespeicherten Indexe als eine 
 * Linine gespeichert. Dann wird die nächste Steigung als Refernzsteigung genommen und der gleiche Prozess erfolgt wieder.
 * Wenn mindenstens 2 Linien erkannt wurden, dann werden aus den vorherig abgespeicherten Indexen der erste und letzte Punkt herausgesucht. Mit diesen wird dann 
 * die Steigung der Linie berechent.
 * @par Linien mergen
 * In einer Schleife werden die dektierten Linien verglichen. Wenn diese gleich sind mit einer Toleranz, dann werden diese zu einer zusammengefügt. Der Anfangspunkt 
 * ist der der ersten Linie und der Endpunkt der der zweiten Linie. Die Steigung der neuen Lininie berechnet sich aus dem arithmetischen Mittel der beiden Steigungen. 
 * Außerdem wird ein Richtungsvektor gebildet.
 * @par Erkennen der Landmarke
 * Das Skalarprodukt zweier "neuen" Linien wird berechnet und deren Beträge. Mit dem arccos(v1*v2/|v1]*|v2|) wird dann der Winkel in Grad berechnet. Wenn dieser mit 
 * dem einzigartigen der Landmarke übereinstimmt wird die ID der Landmarke returniert.
 */
int Filter::detectLandmark()
{
    if(laser_status == false) return 0;

    ROS_INFO("Landmark detection start");
    std::vector<float> k;
    std::vector<int> linien_indexe;
    std::vector<std::vector <int>> linien_indize;
    std::vector<std::vector <float>> linien_vec;
    
    k.resize(360);
    linien_indize.resize(360);
    
    int point_1 = 0;    // Startpunkt
    int step = 1;       // Anzahl an Laserpunkten die übersprungen werden zwischen den Messungen
    int point_2 = 8;   // Abstand zwischen Laserpunkten
                            //Referenzpunkt ist größer, damit Punkte übersprungen werden und die Steigung gemittelt ist
    float k_toleranz = 1.4;         // gibt an wieviel die STeigung abweichen darf um als gleicher Wert akzeptiert zu werden
    float linien_toleranz = 0.9;     // gibt an wieviel die STeigung abweichen darf um als gleicher Wert akzeptiert zu werden

    int match = 0;              // gibt an wieviele Steigungen übereinstimmen
    int no_match = 0;           // gibt an wieviele Steigungen nicht übereinstimmen
    int linien_counter = 0;     // gibt an wieviele Linien erkannt wurden im ersten Schritt
    int needed_matches = 8;         //gibt an wieviele Steigungen ähnlich sein müssen, um eine Linie zu bilden
    int needed_no_matches = 5;      //gibt an wieviele Steigungen anders sein müssen, damit keine Linie erkannt wird
    
    int cnt = 1;                    // Zählvariable mit offset, damit letzter Wert mit Stelle 0 verglichen werden kann und somit der alle Werte verglichen werden


    //-------------- Steigung zwischen Laserpunkten ---------------------//
    // berechnet Steigungen der Laserpunkte, dabei werden immer point_2 -1 ausgelassen
    for(int i = 0; point_1 < laser_val.size(); i++)   
    {
        if(point_2 >= laser_val.size()) point_2 = 0;
        //std::cout << "Laser val " << point_1 << ": " << laser_val.at(point_1) << "\tLaser val " << point_2 << ": " << laser_val.at(point_2) << std::endl;
        if (laser_val.at(point_1) == 333 || laser_val.at(point_2) == 333)   // überspringt invalide Messung
        {
            point_1 += step;
            point_2 += step;
            k.at(i) = 333;
        }else{  // berechnet die Steigung
            k.at(i) = (laser_y.at(point_1) - laser_y.at(point_2)) / (laser_x.at(point_1) - laser_x.at(point_2)); // Steigung zwischen 2 Laserpunkten
            point_1 += step;
            point_2 += step;
        }
        //std::cout << "Steigung k" << i << ": " << k.at(i) << std::endl;
    }
    //-----------------------------------------------------------------//


    //-----------------Linien aus Steigungen generieren----------------------//
        

    // *** Linie aus Steigungen generieren ***//
    for(int i = 0; i < k.size(); i ++)   // Steigung i als Referenzpunkt nehmen
    {
        if(k.at(i) == 333) continue;    //skip invalid ones
        for(int j = 1; j < k.size(); j++) // Referenzsteigung mit nächsten Steigungen vergleichen
        {
            if(cnt >= k.size()) cnt = 0;
            if(k.at(i) - k_toleranz <= k.at(cnt) && k.at(i) + k_toleranz >= k.at(cnt))  // gleiche Steigung in einen Topf werfen
            {
                //std::cout << " match " << j << "\tja"  << std::endl;
                linien_indexe.push_back(i);
                linien_indexe.push_back(cnt);
                match++;
            } else {
                //std::cout << " match " << j  << "\tnein" << std::endl;
                no_match++;  //steigung passt nicht zusammen, no_match erhöhen
            }
            //std::cout << "no matches: " << no_match  << std::endl;

            if(no_match >= needed_no_matches) break; // zu viele nicht passende steigungen, nächste steigung als startkriterium nehmen
            cnt++;
        }

        // *** wenn genügen Steigungen übereinstimmen, dann werden diese als linine zusammengepackt ***//
        if(match >= needed_matches) // ähnlich Steigungen bilden eine Linie
        {
            //std::cout << "Punktpaare abspeichern " << std::endl;
            linien_indize[linien_counter] = linien_indexe;
            linien_counter++;
        } else {
            //std::cout << "keine Punktpaare gefunden" << std::endl;
        }
        //reseten der Variablen
        cnt = 1;
        match = 0;
        no_match = 0;
        linien_indexe.clear();
    }
    //std::cout << "linien counter: " << linien_counter << std::endl;
    
    // *** mindenstens 2 Linien müssen erkannt werden, damit eine Winkelberechnung möglich ist ***//
    if(linien_counter >= 2)
    {
        linien_vec.resize(linien_counter);
        
        // get first and last point of lines[i]
        for(int i = 0; i < linien_counter; i++)
        {
            int p1 = linien_indize.at(i).at(0);  //ersten Punkt der Linie
            int p2 = linien_indize.at(i).back(); //letzer Punkt der Linie
            float lx = laser_x.at(p1);      // x-wert punkt 1
            float ly = laser_y.at(p1);      // y-wert punkt 1
            float lx2 = laser_x.at(p2);     // x-wert punkt 2
            float ly2 = laser_y.at(p2);     // y-wert punkt 2
            
            // Vektoren bilden
            linien_vec[i].resize(3);
            linien_vec[i][0] = p1;  
            linien_vec[i][1] = p2;
            linien_vec[i][2] = ly2-ly / lx2-lx; //Steigung der Linie
            //std::cout << "Vec " << i << ":\tX: " << linien_vec.at(i).at(0) << "\tY: " << linien_vec.at(i).at(1) << "\tk: " << k_linie << std::endl;
        }
        
        //std::cout << "Anzahl Linien vorher : " << linien_vec.size() << std::endl;
        
        // *** merged ähnliche Linien zu einer und mittelt sie dabei ***//
        std::vector<int> merged;
        for(int i = 0; i < linien_vec.size(); i++)
        {
            if(cnt >= linien_vec.size()) cnt = 0;
            //std::cout << "k von " << "Linie " << i << " : " << linien_vec[i][2] << "\tLinie " << cnt << " : " << linien_vec[cnt][2] << std::endl;
            
            // überprüft, ob die Linien ähnliche Steigungen haben
            if(linien_vec[i][2] - linien_toleranz <= linien_vec[cnt][2] &&
                linien_vec[i][2] + linien_toleranz >= linien_vec[cnt][2])
            {
                int p1 = linien_vec[i][0];      // Anfangspunkt von linie 1
                int p2 = linien_vec[cnt][1];    // Endpunkt von linie 2
                //std::cout << "p1 : " << p1 << "\tp2 : " << p2 << std::endl;
                if(p1 < 0 || p2 < 0) return 0;  // damit es nicht abstürtzt, p1 und p2 werden falsch berechnet
                float lx = laser_x.at(p1);      // x-wert punkt 1
                float ly = laser_y.at(p1);      // y-wert punkt 1
                float lx2 = laser_x.at(p2);     // x-wert punkt 2
                float ly2 = laser_y.at(p2);     // y-wert punkt 2
                
                //Vektor bilden
                linien_vec[cnt][0] = lx2 - lx;
                linien_vec[cnt][1] = ly2 - ly;
                linien_vec[cnt][2] = (linien_vec[i][2] + linien_vec[cnt][2]) / 2.0;
                merged.push_back(i);
            }
            cnt++;
        }
        cnt = 1;
        
        std::reverse(merged.begin(), merged.end());    //dreht den vector um
        // remove merged lines from vector
        for(int i = 0; i < merged.size(); i++)
        {
            //std::cout << "merge: " << merged.at(i) << std::endl;
            linien_vec.erase(linien_vec.begin() + merged.at(i));
        }
        //std::cout << "Anzahl Linien nachher : " << linien_vec.size() << std::endl;

        //------------------------------------------------------------------------------//

        //----------------------- Erkennen der Landmarken -------------------//

        // Winkel zwischen Linien berechnen    
        for(int i = 0; i < linien_vec.size(); i++)
        {
            //std::cout << "Linie " << i << ": X: " << linien_vec[i][0] << "\tY: " << linien_vec[i][1] << std::endl;
            if(cnt >= linien_vec.size()) cnt = 0;
            // Skalar und co
            float scalar = linien_vec[i][0] * linien_vec[cnt][0] + linien_vec[i][1] * linien_vec[cnt][1];
            float betrag_v1 = sqrt(pow(linien_vec[i][0],2) + pow(linien_vec[i][1],2));
            float betrag_v2 = sqrt(pow(linien_vec[cnt][0],2) + pow(linien_vec[cnt][1],2));
            // Winkel zw. Linen berechnen
            float winkel = acos(scalar/(betrag_v1 * betrag_v2)) * 180.0/M_PI;   //Winkel in Grad zwischen den Linien
            //std::cout << "   betrag 1: " << i << ": " << betrag_v1 << std::endl;
            //std::cout << "   betrag 2: " << i << ": " << betrag_v2 << std::endl;
            //std::cout << "     winkel: " << i << ": " << winkel << std::endl;
            cnt++;
    
            // Überprüfen ob die Linien mit der Landmarke übereinstimmen 
            if(winkel - 5 <= 130 && winkel + 5 >= 130)
            {
                ROS_INFO("Landmarke erkannt!!!");
                return 1;   //landmarke 1, da es nur eine lm gibt
            }
        }
        cnt = 1;
    }
    //-----------------------------------------------------------------------//
    linien_vec.clear();
    ROS_INFO("Landmark detection failed");
    return 0;
}

/**
 * Die Subscriber und Publisher werden definiert. Die Startpostion des Roboters muss hier eingegeben werden. Speicher für Vektoren wird allokiert. \n
 * Berechnung der Laserwinkel wird einmalig durchgeführt. Da die Winkelabstände immer gleich sind werden diese abgespeichert um später Rechenaufwand einzusparen.
 * @brief Der Konstrukter der Filter Klasse
 */
Filter::Filter()
{
    ros::NodeHandle node("~");
    sub1.subscribe(node, "odom_sub", 1);
    sub2.subscribe(node, "imu_sub", 1);
    sub_laser = node.subscribe("sensor_sub", 1, &Filter::callbackLaser, this); //Laser-Sensorwert
    sync.reset(new Sync(MySyncPolicy(10), sub1, sub2));   
    sync->registerCallback(boost::bind(&Filter::callback, this, _1, _2));

    pose_cov_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_cov_q", 1);

    first_measurement = true;
    x_old = 1.9;            //Startposition x, ungleich 0 ansonsten mathematisches problem
    y_old = 0.00001;        //Startposition y, ungleich 0 ansonsten mathematisches problem
    theta_old = 0.00001;    //Startwinkel theta, ungleich 0 ansonsten mathematisches problem

    M_I.setIdentity();
    M_var.setZero();
    M_var_old.setZero();    //eventuell zu setIdenty wegen multiplikation mit 0

    lm.resize(9);
    laser_val.resize(360);
    laser_angles.resize(360);
    laser_x.resize(360);
    laser_y.resize(360);
    M_Kv.resize(360);
    M_Hv.resize(360);
    V_mueh_dv.resize(360);
    V_muehv.resize(360);
    
    for(int i = 0; i < laser_angles.size(); i++) laser_angles.at(i) = i * (M_PI/ 180.0);    //calc angles [rad] for each measurement (1 deg per measurement)
    //for(int i = 0; i < laser_angles.size(); i++) std::cout << "laser " << i << " :" << laser_angles.at(i) << std::endl;

    readLandmarks();
}

/**
 * Es wird ROS initialisiert und ein Objekt der Filter-Klasse erstellt.
 * @brief Die Main
 *
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf");
    
    Filter Filters;

    ros::spin();
}


/*! \mainpage EKF-Lokalisierung | Salner
* 
* \section Anforderungen
* Gazebo-Welt: Turtlebot3 \n
* Roboter: waffle \n
* Algorithmus: EKF-Lok \n
* Landmarken sind selber zu definieren \n 4 Punkte sind anzufahren
* 
* \section Programmablauf
* Es werden 2 Nodes parallel ausgeführt. Einerseits die Node ekf, welche die EKF-Lokalisierung macht und andererseits die Node move_rob, 
* welche den Roboter fahren lässt. \n
*
* Als Fahrbefehl wurden die Werte der Odometrie genommen. Dabei werden die Geschwindigkeiten genutzt. Die Odmetrie wird als direkter Fahrbefehl an
* die Motoren interpretiert.
*
* \subsection ekf
* Zuerst wird eine Prediction der aktuellen Pose aufgrund der vorherigen Pose und des Fahrbefehls berechent. Dannach wird diese predictete Pose mittels der
* IMU-Messdaten korregiert. Sofern dann eine Landmarke mit dem Laserscan erkannt wird, wird die Laser-Korrektur ausgeführt. Dieser Ablauf wird immer aufgerufen, wenn 
* der synchronisierten Callback aufgerufen wird. Die genaueren Beschreibungen der Mathematik ist der Klasse Filter, bei den jeweiligen Methoden zu finden.
* 
* \subsection move_robot
* Aus einem yaml-file werden defienierte Zielposen ausgelesen und dann gepublisht, so dass der Roboter diese nach der Reihe anfährt.
*
* 
* \section Landmarkenerkennung
* Da die Map symmetrisch um eine Achse ist und die Säulen alle den selben Abstand und Durchmesser haben, wurde ein Eck als Landmarke gewählt, welches einzigartig ist.
* Diese 2 Wände, mit der Länge 0.55 m, bilden einen Winkel von 230° oder 130°. Dadurch wurde der Algorithmus zur Erkennung von dieser Landmarke, so designt dass 
* erkannt wird ob die Laserpunkte auf einer Linie sind. Sofern mindenstens 2 Linien erkannt werden wird der Winkel zwischen diesen berechnet und mit dem der 
* Landmarke verglichen. Die genauere Beschreibung der Landmarkenerkennung ist in der Methode detectLandmark() der Klasse Filter zu finden. \n
* In der Datei landmarks.yaml ist die Landmarke definiert. Der Code ist so geschrieben, dass dieses FIle erweitert werden kann und nur die Überprüfung auf neue
* Winkel oder Beträge eingefügt werden müsste.
* 
* \section Setup
* Für ein funktionierenden Ablauf müssen diese Schritte durchgeführt werden.
*
* \subsection Model
* Damit das richtige Model (turtlebot waffle) geladen wird, muss mit "vi ~/.bashrc" dieses Datei bearbeitet werden. Dort ist das Model zum umstellen. 
* "export = TURTLEBOT3_MODEL=waffle"
* 
* \subsection Pfadplanung
* Damit die Pfadplanung funktioniert muss eventuell das package dwa_local_planner nachinstalliert werden.
 * 
 * 
 * \section Run
 * Um das Programm aufzuführen den Befehl "roslaunch salner start.launch" im Terminal ausführen. Davor muss natürlich das Programm noch mit *catkin_make* kompiliert
 * werden.
 * 
 * 
 */