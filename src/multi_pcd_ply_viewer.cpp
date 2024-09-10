#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vector>
#include <cstdlib>  // for rand() and srand()

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <file1.pcd/.ply> [file2.pcd/.ply] ..." << std::endl;
        return -1;
    }

    // PCL Viewerの作成
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // 色をランダムに生成するためのシードを設定
    srand(time(0));

    // すべてのコマンドライン引数で与えられたPCD/PLYファイルを読み込み、表示
    for (int i = 1; i < argc; ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        std::string filename = argv[i];
        std::string extension = filename.substr(filename.find_last_of('.') + 1);

        // ファイルの拡張子に基づいて適切な読み込み関数を使用
        if (extension == "pcd")
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
            {
                PCL_ERROR("Couldn't read file %s\n", filename.c_str());
                return -1;
            }
        }
        else if (extension == "ply")
        {
            if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1)
            {
                PCL_ERROR("Couldn't read file %s\n", filename.c_str());
                return -1;
            }
        }
        else
        {
            std::cerr << "Unsupported file format: " << filename << std::endl;
            continue;
        }

        std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << filename << std::endl;

        // ランダムな色を生成
        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        // cloudを表示
        std::string cloud_id = "cloud" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, r * 255, g * 255, b * 255);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color_handler, cloud_id);
    }

    // ビューアを表示
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}
