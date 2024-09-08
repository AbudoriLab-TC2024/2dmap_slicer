#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <iostream>

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        PCL_ERROR("Usage: %s <input_file.pcd> <roll_angle_in_degrees> <pitch_angle_in_degrees>\n", argv[0]);
        return -1;
    }

    // 1. 点群を読み込む
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", argv[1]);
        return -1;
    }

    // 2. コマンドライン引数から回転角度を取得
    float roll_angle = std::stof(argv[2]) * M_PI / 180.0f;  // Roll angle in radians
    float pitch_angle = std::stof(argv[3]) * M_PI / 180.0f; // Pitch angle in radians

    // 3. Roll方向の回転行列を作成
    Eigen::Affine3f transform_roll = Eigen::Affine3f::Identity();
    if (roll_angle != 0.0f) {
        transform_roll.rotate(Eigen::AngleAxisf(roll_angle, Eigen::Vector3f::UnitX()));
    }

    // 4. Pitch方向の回転行列を作成
    Eigen::Affine3f transform_pitch = Eigen::Affine3f::Identity();
    if (pitch_angle != 0.0f) {
        transform_pitch.rotate(Eigen::AngleAxisf(pitch_angle, Eigen::Vector3f::UnitY()));
    }

    // 5. 点群をRollとPitchで回転させる
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *rotated_cloud, transform_roll.matrix());
    pcl::transformPointCloud(*rotated_cloud, *rotated_cloud, transform_pitch.matrix());

    // 6. 回転後の点群を保存
    pcl::io::savePCDFileASCII("rotated_cloud.pcd", *rotated_cloud);

    // 7. 可視化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // 元の点群を表示（灰色）
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "original cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0.3, 0.3, "original cloud");

    // 回転後の点群を表示（緑色）
    viewer->addPointCloud<pcl::PointXYZ>(rotated_cloud, "rotated cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "rotated cloud");

    // 座標軸を表示（3mの長さで）
    viewer->addCoordinateSystem(3.0);
    viewer->initCameraParameters();

    // ビューアを閉じるまでループ
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}

