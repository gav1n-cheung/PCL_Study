#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h> // 点云坐标变换
#include <pcl/visualization/pcl_visualizer.h>


void pointUsages(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width=640;//EG1
    cloud->height=480;//EG2

    cloud->width=307200;//EG3
    cloud->height=1;//无组织的点云数据

    pcl::PointCloud<pcl::PointXYZ> cloud1;
    //std::vector<pcl::PointXYZ> data=cloud1.points;

}

