
// Created by cs18 on 5/11/21.

/*一：基本结构
 *   PCL点云库的基本数据类型为PointCloud<pcl::PointCloud>。PointCloud是C++类，其中包含以下的数据字段：
 *     （1）pcl:'width<pcl::PointCloud::width>' (数据类型int)
 *          用点的数量来表示点云数据集的宽度。宽度width有两个含义：
 *              （a）如果是无组织的点云数据集，它可以指定点云中的总数。
 *              （b）也可以指定有组织的点云数据集一行的总点数
 *        Note：有组织的点云数据集---赋予了类似于一个有组织的图像（或矩阵）状的结构的点云数据集。其中，数据被划分为行和列。
 *              这种数据集有可能来自于立体摄像机或飞行时间摄像机获得的数据。
 *              有组织的数据集的优点在于，通过相邻点之间的关系，最临近操作的操作更加有效，从而加快了计算速度并且降低了PCL中某些算法的成本。
 *        Note：可投影点云---指根据针孔相机模型，在有组织的点云中点的(u,v)索引与实际3D值之间具有相关性。这种相关性可以以最简单的方法表示为
 *               u = f*x/z 和 v = f*y/z 。
 *        EG:见MatrixTrans.cpp
 *
 *      (2)pcl:'height<pcl::PointCloud::height>' (数据类型int)
 *          用点的数量来表示点云数据集的高度。高度height有两个含义：
 *              （a）如果是无组织的点云数据集，将其置为1，可以标明数据集是否已经被组织。
 *              （b）也可以指定有组织的点云数据集的总行数（高度）
 *        EG:见MatrixTrans.cpp
 *        EG:见MatrixTrans.cpp
 *      (3)pcl:'points<pcl::PointCloud>' (数据类型 std::vector<PointT>)
 *          储存所有PointT类型的所有点的向量数组。例如：对于包含XYZ数据的点云，points包含pcl::PointXYZ所有元素的向量。
 *        EG:见MatrixTrans.cpp
 *      (4)pcl:'is_dense<pcl::PointCloud::is_dense>' （数据类型 bool）
 *          指定点云所有数据是否都是有限的(是则为ture)，或者某些点的XYZ值有可能包含Inf（下确界）/NaN值（未定值或不可表示的值）（是则为false）
 *      (5)pcl:'sensor_origin_<pcl::PointCloud::sensor_origin_>' (数据类型 Eigen::Vector4f)
 *          指定传感器采集的姿势（固定/平移）。该变量通常是可选的，并且在PCL中的大多数算法中都不使用。
 *      (6)pcl:'sensor_orientation_<pcl::PointCloud::sensor_orientation_>' (数据类型 Eigen::Quaternionf)
 *          指定传感器采集姿势（方法）。该变量成员通常是可选的，并且在PCL中的大多数算法都不使用。
 *
 *      为了简化开发，PointCloud<pcl::PointCloud>类包含许多帮助程序成员函数。例如，用户不必检查代码中的height是否为1就可以查看数据集是否是组织好的，
 *      而可以使用 pcl:'PointCloud<pcl::PointCloud::isOrganized>'。
 *          EG: 见MatrixTrans.cpp
 *      所述的PointT类型是主点数据类型，并且描述了每个单独点的类型。PCL中有许多点类型。
 */

/*二：使用矩阵变换点云
 * 在这一部分，我们将学习如何使用4*4矩阵来变换矩阵。将旋转和平移操作应用于已有的点云数据中，然后展示其结果。
 * 该程序能够加载一个PCD或PLY文件。我们在其上应用矩阵变换，并且显示原始的点云和变换后的点云。
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */




