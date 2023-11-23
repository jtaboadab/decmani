#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main()
{
    // Matriz de proyección
    float fx = 509.0920104980469; 
    float fy = 509.0920104980469; 
    float cx = 299.10101318359375; 
    float cy = 244.7949981689453; 

    // Creamos las variables PCL
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr voxel_cloud (new pcl::PCLPointCloud2());
    pcl::PCDReader cloud_reader;
    pcl::PCDWriter cloud_writer;

    std::string path="/home/tfg/dectmani_ws/src/pcl_cpp/src/";

    // Leemos la nube de puntos
    cloud_reader.read(path+std::string("cloud.pcd"), *cloud);

    std::cout << "Source Cloud Points " << cloud->width * cloud->height << std::endl;

    // Filtramos para quedarnos con un menmor número de puntos
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.05, 0.05, 0.05);
    voxel_filter.filter(*voxel_cloud);

    std::cout << "Source Cloud Points " << voxel_cloud->width * voxel_cloud->height << std::endl;

    // Guardamos la nube filtrada
    cloud_writer.write(path+std::string("voxelized.pcd"), *voxel_cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

    // Cargamos las imágenes
    cv::Mat im_color = cv::imread(path+std::string("imagen.jpg"));
    cv::Mat im_mask = cv::imread(path+std::string("mask.jpg"));

    // Visualizamos la nube de puntos en la imagen a color
    cv::namedWindow("PointCloud on Image", cv::WINDOW_NORMAL);

    // Recorremos los puntos de la nube
    for (int i = 0; i < voxel_cloud->width * voxel_cloud->height; ++i) {
        // Obtenemos las coordenadas 3D del punto
        float x = voxel_cloud->data[i * voxel_cloud->point_step + voxel_cloud->fields[0].offset + 0];
        float y = voxel_cloud->data[i * voxel_cloud->point_step + voxel_cloud->fields[0].offset + 4];
        float z = voxel_cloud->data[i * voxel_cloud->point_step + voxel_cloud->fields[0].offset + 8];

        // Calculamos el tamaño del círculo en función de la distancia
        int radio = std::max(static_cast<int>(5.0f / z), 0); 

        // Convertimos las coordenadas 3D a píxeles en la imagen
        int px = static_cast<int>(x * fx / z + cx);
        int py = static_cast<int>(y * fy / z + cy);

        // Dibujamos un círculo en la imagen en cada punto
        cv::circle(im_color, cv::Point(px, py), radio, cv::Scalar(0, 255, 0), -1);
    }

    // Mostramos la imagen con la nube de puntos
    cv::imshow("PointCloud on Image", im_color);
    cv::waitKey(0);

    return(0);
}