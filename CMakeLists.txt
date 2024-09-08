cmake_minimum_required(VERSION 3.0.2)
project(2dmap_slicer)

# 依存ライブラリ
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)

# 実行ファイルを追加
add_executable(remove_ground src/remove_ground.cpp)
add_executable(rotate_cloud src/rotate_cloud.cpp)

# OpenCVライブラリをリンク
#target_link_libraries(pcl_example ${OpenCV_LIBS})

# PCLライブラリをリンク
target_link_libraries(remove_ground ${PCL_LIBRARIES})
target_link_libraries(rotate_cloud ${PCL_LIBRARIES})

# インクルードディレクトリを設定
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# ライブラリディレクトリを設定
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

