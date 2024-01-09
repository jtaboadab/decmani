#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/point.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

using std::placeholders::_1;

class Calibration : public rclcpp::Node {
public:
    Calibration()
        : Node("calibration") {
        // Publishers
        publisher_center_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/calibration/center_point", 10);

        // Configuración del suscriptor de imagen
        subscription_color_image_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", 10, std::bind(&Calibration::listener_callback_color_image, this, _1));
        
        subscription_center_point_ = this->create_subscription<geometry_msgs::msg::Point>("/bbox/center_point", 10, std::bind(&Calibration::listener_callback_center_point, this, _1));

        // Configuración del broadcaster TF
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Configuración d el diccionario ArUco
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        // Configuración de los parámetros de la cámara
        cameraMatrix = (cv::Mat_<double>(3, 3) << 509.0920104980469, 0, 299.10101318359375, 0, 509.0920104980469, 244.7949981689453, 0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

        // Tamaño del ArUco
        markerLength = 0.075;

        // Configuración del sistema de coordenadas
        objPoints = cv::Mat(4, 1, CV_32FC3);
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
    }

private:
    void listener_callback_color_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convertir mensaje a imagen OpenCV
        cv_bridge::CvImageConstPtr im_color_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat im_color = im_color_msg->image;

        // Detectar marcadores ArUco
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(im_color, dictionary, corners, ids);

        // Si se ha detectado el marcador
        if (!ids.empty()) {

            RCLCPP_INFO(this->get_logger(), "si!");

            // Dibujar marcadores
            cv::aruco::drawDetectedMarkers(im_color, corners, ids);

            cv::Mat rvecs, tvecs;
            // Obtenemos los parámetros extrínsecos que relacionan cámara y ArUco
            cv::solvePnP(objPoints, corners[0], cameraMatrix, distCoeffs, rvecs, tvecs);

            // Dibujar ejes para cada marcador
            cv::aruco::drawAxis(im_color, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1);
            
            // Mostrar imagen
            cv::imshow("out", im_color);
            (char)cv::waitKey(100);

            // Publicar transformación del marcador ArUco
            if (!rvecs.empty()) {
                publish_aruco_tf(rvecs, tvecs, ids[0]);
            }

        }
        else{

            RCLCPP_INFO(this->get_logger(), "no!");
        }

    }

    void listener_callback_center_point(const geometry_msgs::msg::Point::SharedPtr msg) {

        center_point_x = msg->x;
        center_point_y = msg->y;
        center_point_z = msg->z;

    }

    void publish_aruco_tf(const cv::Mat& rotation_vector_aruco, const cv::Mat& translation_vector_aruco, int id) {

        geometry_msgs::msg::TransformStamped t_aruco;

        // Pasamos de matriz de rotación a cuaternios
        cv::Matx33d R;
        cv::Rodrigues(rotation_vector_aruco, R);
        Eigen::Matrix3d wRo;
        wRo << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);

        Eigen::Matrix4d T;
        T.topLeftCorner(3, 3) = wRo;
        T.col(3).head(3) << translation_vector_aruco.at<double>(0), 
        translation_vector_aruco.at<double>(1), 
        translation_vector_aruco.at<double>(2);
        T.row(3) << 0, 0, 0, 1;

        Eigen::Matrix3d rot = T.block(0, 0, 3, 3);
        Eigen::Quaternion<double> rot_quaternion(rot);

        // Configuramos la relación entre cámara y ArUco
        t_aruco.header.stamp = this->now();
        t_aruco.header.frame_id = "camera_color_optical_frame";
        t_aruco.child_frame_id = "aruco_frame_" + std::to_string(id);

        t_aruco.transform.translation.x = T(0,3);
        t_aruco.transform.translation.y = T(1,3);
        t_aruco.transform.translation.z = T(2,3);

        t_aruco.transform.rotation.x = rot_quaternion.x();
        t_aruco.transform.rotation.y = rot_quaternion.y();
        t_aruco.transform.rotation.z = rot_quaternion.z();
        t_aruco.transform.rotation.w = rot_quaternion.w();

        // Configuramos la relación entre ArUco y manipulador
        geometry_msgs::msg::TransformStamped t_robot;

        t_robot.header.stamp = this->now();
        t_robot.header.frame_id = "aruco_frame_" + std::to_string(id);
        t_robot.child_frame_id = "robot";

        t_robot.transform.translation.x = -0.015;
        t_robot.transform.translation.y = 0.155;
        t_robot.transform.translation.z = 0;

        tf_broadcaster_->sendTransform(t_aruco);
        tf_broadcaster_->sendTransform(t_robot);

        // Convertimos el punto central del vaso a formato PointStamped
        geometry_msgs::msg::PointStamped punto_camara;
        punto_camara.header.frame_id = "camera_color_optical_frame";
        punto_camara.point.x = center_point_x;
        punto_camara.point.y = center_point_y;
        punto_camara.point.z = center_point_z;

        publisher_center_point_->publish(punto_camara); 
        
    }

    // Variables
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_center_point_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_color_image_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_center_point_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Mat cameraMatrix, distCoeffs, objPoints;
    float markerLength;
    float center_point_x;
    float center_point_y;
    float center_point_z;
};

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Calibration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    
}