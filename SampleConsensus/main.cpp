//如何使用随机样本共识模型

//在本教程中，我们将学习如何使用带有平面模型的RandomSampleConsensus来获得适合该模型的云。

/*理论入门
 * "RANdom SAmple Consensus"的缩写是RANSAC，他是一种迭代方法，用于从包含异常值的一组数据中估计
 * 数学模型的参数。该算法假设我们正在查看的数据都由内点和外点组成。内点可以通过一组特定参数值的模型
 * 来解释，而异常值在任何情况下都不适合该模型。另一个必要的假设是有一个程序可以从数据中最佳的估计所选
 * 模型的参数
 RANSAC算法的输入是一组观测数据,一个可以解释或拟合观测的参数化模型，以及一些置信参数。
 RANSAC通过迭代选择原始数据的随机子集来实现其目标。这些数据是假设的内部数据，然后按照如下方式测试该
 假设：
    （1）模型被拟合到假设的内点，即模型的所有自由参数都是从内点重建的。
    （2）然后针对拟合模型测试所有其他数据，如果某个点与估计模型拟合良好，则也将其视为假设的内点。
    （3）如果足够多的点被归类为假设的内点，则估计模型相当好。
    （4）该模型是从所有假设的内点重新估计的，因为他只是从初始的假设内点集估计出来的。
    （5）最后，通过估计内点相对于模型的误差来评估模型。
 这个过程重复固定的次数，每次都会产生一个模型，因为太少的点被归类为内点而被拒绝，或者是一个带有相应
 误差度量的精炼模型。在后一种情况下，如果其误差低于上次保存的模型，我们将保留精制模型。

 RANSAC的一个优点是它能够对模型参数进行鲁棒估计，即就算是数据集中存在大量异常值，他也可以高度准确
 地估计参数。RANSAC的一个缺点是计算这些参数所需的时间没有上限。当计算的迭代次数有限时，获得的解可能不是
 最佳的，甚至可能不是很好地拟合数据的解。通过这种方式，RANSAC提供了一种权衡；通过计算更多的迭代
 次数，可以增加生成合理模型的概率。RANSAC的另一个缺点是它需要设置特定于问题的阈值。
 RANSAC只能为特定数据集估计一个模型。对于存在两个（或更多）模型的任何一种模型方法，RANSAC可能
 无法找到任何一个。
 （见https://pcl.readthedocs.io/projects/tutorials/en/latest/random_sample_consensus.html#random-sample-consensus）
拟合结果图
 */
#include <iostream>
#include <thread>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud,"sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"sample cloud");
    viewer->initCameraParameters();
    return (viewer);
}
int main(int argc,char** argv){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
//初始化两个PointClouds并用点填充其中之一，这些点中的大多数根据模型防止在云中，但其中一小部分（1/5）被赋予任意位置。
    cloud->width=500;
    cloud->height=1;
    cloud->is_dense= false;
    cloud->points.resize(cloud->width*cloud->height);
    for (int i = 0; i < cloud->size(); ++i) {
        if (pcl::console::find_argument (argc, argv, "-s") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
        {
            (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0);
            (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0);
            if (i % 5 == 0)
                (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0);
            else if(i % 2 == 0)
                (*cloud)[i].z =  sqrt( 1 - ((*cloud)[i].x * (*cloud)[i].x)
                                       - ((*cloud)[i].y * (*cloud)[i].y));
            else
                (*cloud)[i].z =  - sqrt( 1 - ((*cloud)[i].x * (*cloud)[i].x)
                                         - ((*cloud)[i].y * (*cloud)[i].y));
        }
        else
        {
            (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0);
            (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0);
            if( i % 2 == 0)
                (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0);
            else
                (*cloud)[i].z = -1 * ((*cloud)[i].x + (*cloud)[i].y);
        }
    }
    //接下来，我们创建一个整数向量，它可以存储来自PointCloud的内点的位置，
    // 现在我们可以使用来自输入点云的平面或球体模型构建我们的RandomSampleConsensus对象
    std::vector<int> inliers;
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    if (pcl::console::find_argument(argc,argv,"-f")>=0){
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(0.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    } else if (pcl::console::find_argument(argc,argv,"-sf")>=0){
        pcl::RandomSampleConsensus<pcl::PointXYZ>ransac(model_s);
        ransac.setDistanceThreshold(0.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }
    //将我们的结果给到另一个点云，我们可以通过查看器来查看原始或处理过的点云
    pcl::copyPointCloud(*cloud,inliers,*final);
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if (pcl::console::find_argument(argc,argv,"-f")>=0||pcl::console::find_argument(argc,argv,"-sf")>=0)
        viewer= simpleVis(final);
    else
        viewer= simpleVis(cloud);
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
    return 0;

}




