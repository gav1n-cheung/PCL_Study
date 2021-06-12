////
//// Created by cheung on 2021/6/12.
////
///*基于最小切割的分割算法
// * 在本教程中，我们将学习如何使用pcl::MinCutSegmentation课堂中实现的基于最小切割的分割算法
// * 该算法对给定的输入云进行二进制分割。以对象为中心及其半径，算法将云分为两组：前景和背景点（属于对象的点和不属于对象的点）
// */
///*理论入门
// 该算法的思想如下：
// （1）对于给定的点云算法构建的图包含云的每个点作为一组顶点和另外两个称为源和汇的点。图中对应于该点的每个顶点都与源和汇
//        见http://shichaoxin.com/2018/10/26/%E5%9B%BE%E5%83%8F%E5%88%86%E5%89%B2-%E6%9C%80%E5%A4%A7%E6%B5%81-%E6%9C%80%E5%B0%8F%E5%89%B2-%E7%AE%97%E6%B3%95/
// （2）算法为每条边分配权重。有三种布偶能够类型的权重
//      首先，它为云点之间的边缘分配权重，这个权重被称为平滑成本，由以下公式计算
//        见https://pcl.readthedocs.io/projects/tutorials/en/latest/min_cut_segmentation.html#min-cut-segmentation
//        公式一
//        这是点之间的距离。点离得越远，边缘被切割的可能就越大。
//      下一步，算法设置数据成本。他由前景和背景惩罚组成。第一个是将点和源顶点连接起来并具有恒定用户定义值得那些边的权重。第二个
//       分配给连接点与汇顶点的边，并通过
//       https://pcl.readthedocs.io/projects/tutorials/en/latest/min_cut_segmentation.html#min-cut-segmentation
//       公式二计算
//       这里距中心距离是到水平平面中对象的预期中心的距离
//       https://pcl.readthedocs.io/projects/tutorials/en/latest/min_cut_segmentation.html#min-cut-segmentation
//       公式三
//       公式种出显得半径是该算法的输入参数，大致可以认为是从物体中心到其外没有属于前景的点的范围（物体水平半径）
//（3）在所有准备工作之后，搜索最小切口。基于对这个切割的分析，点云被分为前景点和背景点。
// */
//#include <iostream>
//#include <vector>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/segmentation/min_cut_segmentation.h>
//
//int main (int argc, char** argv)
//{
//    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
//    if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("min_cut_segmentation_tutorial.pcd", *cloud) == -1 )
//    {
//        std::cout << "Cloud reading failed." << std::endl;
//        return (-1);
//    }
//    //这几行并不是必须的，他只是表明pcl::MinCutSegmentation类可以使用索引
//    pcl::IndicesPtr indices (new std::vector <int>);
//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud (cloud);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0.0, 1.0);
//    pass.filter (*indices);
//    //实例化pcl::MinCutSegmentation<pcl::PointXYZ>，只有一个参数--点的类型
//    pcl::MinCutSegmentation<pcl::PointXYZ> seg;
//    //设定分割的目标点云和索引
//    seg.setInputCloud (cloud);
//    seg.setIndices (indices);
//    //算法需要已知为对象中心的点。下面的代码规定并设定了该点
//    pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
//    pcl::PointXYZ point;
//    point.x = 68.97;
//    point.y = -18.55;
//    point.z = 0.57;
//    foreground_points->points.push_back(point);
//    seg.setForegroundPoints (foreground_points);
//    //设定sigma和平滑成本计算所需的对象半径
//    seg.setSigma (0.25);
//    seg.setRadius (3.0433856);
//    //临近点的阈值设置
//    seg.setNumberOfNeighbours (14);
//    //设置前景惩罚
//    seg.setSourceWeight (0.8);
//    //启动算法，分割后的簇将包含结果
//    std::vector <pcl::PointIndices> clusters;
//    seg.extract (clusters);
//
//    std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;
//
//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
//    pcl::visualization::CloudViewer viewer ("Cluster viewer");
//    viewer.showCloud(colored_cloud);
//    while (!viewer.wasStopped ())
//    {
//    }
//
//    return (0);
//}
//
