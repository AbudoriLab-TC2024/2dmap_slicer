#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <input_file.ply or .pcd> normal_z threshold(0.0 ~ 1.0)" << std::endl;
        return -1;
    }

    std::string filename = argv[1];
    float ground_normal_z = std::stof(argv[2]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (filename.substr(filename.find_last_of(".") + 1) == "ply")
    {
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1)
        {
            std::cerr << "Error loading PLY file: " << filename << std::endl;
            return -1;
        }
    }
    else if (filename.substr(filename.find_last_of(".") + 1) == "pcd")
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
        {
            std::cerr << "Error loading PCD file: " << filename << std::endl;
            return -1;
        }
    }
    else
    {
        std::cerr << "Unsupported file format. Use .ply or .pcd files." << std::endl;
        return -1;
    }

    std::cout << "Loaded point cloud with " << cloud->width * cloud->height << " points." << std::endl;

    // 法線ベクトルの計算
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setKSearch(50);  // 近傍点の数
    ne.compute(*normals);

    // 法線ベクトルが水平に近い点を除去
    pcl::PointIndices::Ptr non_ground_indices(new pcl::PointIndices);
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);  // 除去された点群のインデックス
    for (size_t i = 0; i < normals->points.size(); ++i)
    {
        if (std::fabs(normals->points[i].normal_z) < ground_normal_z)
        {
            non_ground_indices->indices.push_back(i);
        }
        else
        {
            ground_indices->indices.push_back(i);  // 水平に近い点を別に保存
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(non_ground_indices);
    extract.setNegative(false);
    extract.filter(*non_ground_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setIndices(ground_indices);
    extract.filter(*ground_cloud);

    std::cout << "PointCloud after removing near-horizontal normals: " << non_ground_cloud->width * non_ground_cloud->height << " points." << std::endl;

    // 結果の保存
    pcl::io::savePCDFileASCII("../pcds/non_ground_cloud.pcd", *non_ground_cloud);
    std::cout << "Filtered point cloud saved as 2dmap_slicer/pcds/non_ground_cloud.pcd" << std::endl;
    pcl::io::savePCDFileASCII("../pcds/ground_cloud.pcd", *ground_cloud);
    std::cout << "Ground point cloud saved as 2dmap_slicer/pcds/ground_cloud.pcd" << std::endl;

    // 可視化の設定
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // 除去された点群を赤色で表示
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ground_color(ground_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(ground_cloud, ground_color, "ground cloud");

    // 処理後の点群を緑色で表示
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> non_ground_color(non_ground_cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(non_ground_cloud, non_ground_color, "non-ground cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ground cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "non-ground cloud");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}

