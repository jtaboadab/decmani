#include <memory>
#include <iostream>
#include <vector>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>    

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "msg_srv_creator/msg/mask.hpp"
#include "msg_srv_creator/msg/array.hpp"
#include "msg_srv_creator/msg/arrayofarray.hpp"
#include "msg_srv_creator/msg/arrayuint.hpp"
#include "msg_srv_creator/msg/arrayofarrayuint.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/organized_fast_mesh.h>

using std::placeholders::_1;

msg_srv_creator::msg::Mask mask2D;
msg_srv_creator::msg::Arrayofarrayuint masks_points_2D;
sensor_msgs::msg::Image im_color;
sensor_msgs::msg::Image im_depth;
std::string path="/home/tfg/dectmani_ws/src/pcl_cpp/src/";

class PCLNode : public rclcpp::Node
{
public:
    PCLNode()
        : Node("pclnode")
    {
        publisher_color_image_ = this->create_publisher<sensor_msgs::msg::Image>("/pcl/color_image", 10);
        publisher_depth_image_ = this->create_publisher<sensor_msgs::msg::Image>("/pcl/depth_image", 10);
        publisher_objetcs_points_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl/objects_points_cloud", 10);
        publisher_masks_points_2d_ = this->create_publisher<msg_srv_creator::msg::Arrayofarrayuint>("/pcl/masks_points_2D", 10);
        publisher_masks_image_ = this->create_publisher<msg_srv_creator::msg::Mask>("/pcl/masks_image", 10);


        subscription_color_image_ = this->create_subscription<sensor_msgs::msg::Image>("/detectron2/color_image", 10, std::bind(&PCLNode::listener_callback_color_image, this, _1));
        subscription_depth_image_ = this->create_subscription<sensor_msgs::msg::Image>("/detectron2/depth_image", 10, std::bind(&PCLNode::listener_callback_depth_image, this, _1));
        subscription_objetcs_points_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/detectron2/points_cloud", 10, std::bind(&PCLNode::listener_callback_objetcs_points_cloud, this, _1));
        subscription_masks_image_ = this->create_subscription<msg_srv_creator::msg::Mask>("/detectron2/masks_image", 10, std::bind(&PCLNode::listener_callback_masks_image, this, _1));
        subscription_masks_points_2d_ = this->create_subscription<msg_srv_creator::msg::Arrayofarrayuint>("/detectron2/masks_points_2d", 10, std::bind(&PCLNode::listener_callback_masks_points_2d, this, _1));
        subscription_masks_points_3d_ = this->create_subscription<msg_srv_creator::msg::Arrayofarray>("/detectron2/masks_points_3d", 10, std::bind(&PCLNode::listener_callback_masks_points_3d, this, _1));
    }

private:
    void listener_callback_color_image(const sensor_msgs::msg::Image &msg) const
    {
        im_color = msg;
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(im_color, sensor_msgs::image_encodings::BGR8);
        cv::Mat imagen = cv_ptr->image;
        cv::imwrite("/home/tfg/dectmani_ws/src/pcl_cpp/src/imagen.jpg", imagen);
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data);
    }

    void listener_callback_depth_image(const sensor_msgs::msg::Image &msg) const
    {
        im_depth = msg;
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(im_depth, "");
        cv::Mat imagen = cv_ptr->image;
        cv::imwrite("/home/tfg/dectmani_ws/src/pcl_cpp/src/depth.jpg", imagen);
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data);
    }

    void listener_callback_objetcs_points_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        /*// Convertir el mensaje a una nube de puntos de PCL
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        // Filtrado de la nube de puntos usando VoxelGrid
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.025f, 0.025f, 0.025f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*filtered_cloud);

        // Create a 2D mask
        cv::Mat mask = cv::Mat::zeros (480, 640, CV_8UC1);
        cv::rectangle (mask, cv::Point (100, 100), cv::Point (200, 200), cv::Scalar (255), -1);

        // Apply the mask to the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr masked_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        //masked_cloud -> resize(filtered_cloud->size());

        for (int i = 0; i < masked_cloud->size (); i++)
        {
            if (mask.at<uchar>(filtered_cloud->at (i).y, filtered_cloud->at (i).x) == 0)
            {
                masked_cloud->at (i).x = filtered_cloud->at (i).x;
                masked_cloud->at (i).y = filtered_cloud->at (i).y;
                masked_cloud->at (i).z = filtered_cloud->at (i).z;
            }
        }

        RCLCPP_INFO(this->get_logger(), "filtered cloud size: %li", filtered_cloud->size());
        RCLCPP_INFO(this->get_logger(), "masked cloud size: %li", masked_cloud->size());

        // Generate an organized mesh
        pcl::OrganizedFastMesh<pcl::PointXYZ>::Ptr orgMesh (new pcl::OrganizedFastMesh<pcl::PointXYZ> ());
        pcl::PolygonMesh triangles;
        orgMesh->setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_RIGHT_CUT);
        orgMesh->setInputCloud (masked_cloud);
        orgMesh->reconstruct (triangles);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
        // Convertir el mensaje de imagen a una imagen OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(mask2D.mask_images[0], "");
            // Acceder a la máscara 2D
            cv::Mat mask2D_cv = cv_ptr->image;

            for (size_t i = 0; i < filtered_cloud->size(); ++i) {
                // Acceder a los puntos utilizando índices lineales
                roi_cloud->points.push_back(filtered_cloud->at(i));
            }

        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error al convertir mensaje de imagen: %s", e.what());
            return;
        }

        // Aplicar un filtro passthrough para limitar la región de interés
        pcl::PointCloud<pcl::PointXYZ>::Ptr segment_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(roi_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 2.0);  // Ajusta los límites según tu escenario
        pass.filter(*segment_cloud);*/
        pcl::PCDReader cloud_reader;
        pcl::PCDWriter cloud_writer;

        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*msg, pcl_pc);
        cloud_writer.write(path+std::string("cloud.pcd"), pcl_pc, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc, *pcl_pc_xyz);

        // Filtrado de la nube de puntos usando VoxelGrid
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(pcl_pc_xyz);
        sor.setLeafSize(0.025f, 0.025f, 0.025f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*filtered_cloud);

        RCLCPP_INFO(this->get_logger(), "pcl cloud size: %li", filtered_cloud->size());

        /*cv_bridge::CvImagePtr cv_mask;
        cv_mask = cv_bridge::toCvCopy(mask2D.mask_images[0], sensor_msgs::image_encodings::MONO8);

        RCLCPP_INFO(this->get_logger(), "leegúeaqui");

        for (size_t i = 0; i < filtered_cloud->points.size(); ++i)
        {
            int x = (int)filtered_cloud->points[i].x;
            int y = (int)filtered_cloud->points[i].y;

            if (cv_mask->image.at<uchar>(y,x) == 255)
            {
                object_cloud->points.push_back(filtered_cloud->points[i]);
            }
        }


        RCLCPP_INFO(this->get_logger(), "pcl cloud remove size: %li", filtered_cloud->size());*/

        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(*filtered_cloud, ros_cloud);
        publisher_objetcs_points_cloud_->publish(ros_cloud);
        publisher_masks_points_2d_ ->publish(masks_points_2D);
        publisher_masks_image_ ->publish(mask2D);

        // Imprimir información
        //RCLCPP_INFO(this->get_logger(), "PointCloud after filtering and segmentation has %ld inliers.", inliers->indices.size());

    }

    void listener_callback_masks_image(const msg_srv_creator::msg::Mask &msg) const
    {
        mask2D = msg;
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(mask2D.mask_images[0], sensor_msgs::image_encodings::MONO8);
        cv::Mat imagen = cv_ptr->image;
        cv::imwrite("/home/tfg/dectmani_ws/src/pcl_cpp/src/mask.jpg", imagen);
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.mask_images);
    }

    void listener_callback_masks_points_3d(const msg_srv_creator::msg::Arrayofarray &msg) const
    {
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data);
    }

    void listener_callback_masks_points_2d(const msg_srv_creator::msg::Arrayofarrayuint &msg) const
    {
        masks_points_2D = msg;
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data);
    }

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_color_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_image_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_objetcs_points_cloud_;
    rclcpp::Publisher<msg_srv_creator::msg::Arrayofarrayuint>::SharedPtr publisher_masks_points_2d_;
    rclcpp::Publisher<msg_srv_creator::msg::Mask>::SharedPtr publisher_masks_image_;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_color_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth_image_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_objetcs_points_cloud_;
    rclcpp::Subscription<msg_srv_creator::msg::Mask>::SharedPtr subscription_masks_image_;
    rclcpp::Subscription<msg_srv_creator::msg::Arrayofarrayuint>::SharedPtr subscription_masks_points_2d_;
    rclcpp::Subscription<msg_srv_creator::msg::Arrayofarray>::SharedPtr subscription_masks_points_3d_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

