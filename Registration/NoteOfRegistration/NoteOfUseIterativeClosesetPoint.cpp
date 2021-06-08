//
// Created by cheung on 2021/6/8.
//
/*如何使用迭代最近点法
 *
 * 本文档演示了如何使用迭代最近点法，该算法可以通过最小化两个点云的点之间的距离并对其进行刚性
 * 变换来确定一个点云是否只是另一个点云的刚性变换
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int mainFun1(int argc,char** argv){
    //创建两个pcl::PointCloud<pcl::PointXYZ> boost共享指针并且初始化它们。每个点的类型在pcl命名空间中设置为PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    //填充输入点云
    for (auto& point:*cloud_in ) {
        point.x=1024*rand()/(RAND_MAX+1.0f);
        point.y=1024*rand()/(RAND_MAX+1.0f);
        point.z=1024*rand()/(RAND_MAX+1.0f);
    }
    std::cout<<"Saved "<<cloud_in->size()<<" data points to input:"<<std::endl;
    for (auto& point:*cloud_in) std::cout<<point<<std::endl;
    *cloud_out=*cloud_in;
    std::cout<<"Transformed "<<cloud_in->size()<<" data points:"<<std::endl;
    //对点云执行简单的刚性变换并且再次输出数据
    for(auto& point :*cloud_out ) point.x+=0.7f;
    std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;
    for(auto& point :*cloud_out )  std::cout<<point<<std::endl;
    //创建一个IterativeClosestPoint实例并且设定其参数
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    //创建一个点云对象作为最后的结果容器
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    //如果两个点云正确对齐（意味着它们都是同一个云，只是对其中一个应用了某种刚性变换）,则icp.hasConverged为1(true)，
    //然后输出最后转换的适应度分数以及一些关于它的信息。
    std::cout<<"has converged:"<<icp.hasConverged()<<"score:"<<
    icp.getInputSource()<<std::endl;
    std::cout<<icp.getFinalTransformation()<<std::endl;

    return (0);
}

