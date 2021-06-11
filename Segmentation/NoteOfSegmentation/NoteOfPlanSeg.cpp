////
//// Created by cheung on 2021/6/11.
////
///*在本教程中，我们将学习如何对一组点进行简单的平面分割，即找到支持平面模型的点云中的所有点。
// */
//#include <iostream>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//
//int main(int argc,char** argv){
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    cloud->width=15;
//    cloud->height=1;
//    cloud->points.resize(cloud->width*cloud->height);
//    for (auto& point: *cloud) {
//        point.x=1024*rand()/(RAND_MAX+1.0f);
//        point.y=1024*rand()/(RAND_MAX+1.0f);
//        point.z=1.0f;
//    }
//    (*cloud)[0].z=2.0;
//    (*cloud)[3].z=-2.0;
//    (*cloud)[6].z=4.0;
//
//    std::cerr<<"Point cloud data:"<<cloud->size()<<" points"<<std::endl;
//    for (const auto& point: *cloud)
//        std::cerr << "    " << point.x << " "
//                  << point.y << " "
//                  << point.z << std::endl;
//    //创建了pcl::SACSegmentation<pcl::PointXYZ>对象并设置模型的方法类型。这里也是我们制定距离阈值的地方，
//    //它规定了一个点必须离模型多近才能被视为内点。在本教程中，我们将使用RANSAC方法作为选择的稳健估计器。
//    //由于RANSAC的简单性，我们选定RANSAC选择该方法。
//    pcl::ModelCoefficients::Ptr coefficient (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    seg.setOptimizeCoefficients(true);
//    seg.setModelType(pcl::SACMODEL_PLANE);
//    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setDistanceThreshold(0.01);
//    seg.setInputCloud(cloud);
//    seg.segment(*inliers,*coefficient);
//    if (inliers->indices.size()==0){
//        PCL_ERROR("Could not estimate a planar model for the given dataset");
//        return (-1);
//    }
//    std::cerr<<"Model inliers: "<<inliers->indices.size()<<std::endl;
//    for (size_t i = 0; i < inliers->indices.size(); ++i) {
//        for (const auto& idx:inliers->indices) {
//            std::cerr << idx << "    " << cloud->points[idx].x << " "
//                      << cloud->points[idx].y << " "
//                      << cloud->points[idx].z << std::endl;
//        }
//    }
//    return (0);
//
//}
//
//
