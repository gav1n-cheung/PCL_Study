#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//int main(int argc,char** argv) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd",*cloud)==-1){
//        PCL_ERROR("Could not read the pcd file");
//        return -1;
//    }
//    std::cout<<"loaded"
//            <<cloud->width*cloud->height
//            <<" data points from test_pcd.pcd with the following field:"
//            <<std::endl;
//    for (const auto& point:*cloud) {
//        std::cout<<"x: "<<point.x
//            <<"y: "<<point.y
//            <<"z:"<<point.z<<std::endl;
//    }
//    return 0;
//}
int main(int argc,char** argv){
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.height=10;
    cloud.width=10;
    cloud.is_dense= false;
    cloud.resize(cloud.height*cloud.width);

    for (auto &point:cloud) {
        point.x=1024*rand()/(RAND_MAX+1.0f);
        point.y=1024*rand()/(RAND_MAX+1.0f);
        point.z=1024*rand()/(RAND_MAX+1.0f);
    }
    pcl::io::savePCDFileASCII("test.pcd",cloud);
    std::cout<<"Saved "<<cloud.size()<<" data points to test.pcd"<<std::endl;
    for (const auto point:cloud) {
        std::cout<<" "<<point.x
            <<" "<<point.y
            <<" "<<point.z<<std::endl;
    }
    return 0;

}
