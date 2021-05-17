//
// Created by cs18 on 5/17/21.
//
/*全局对齐的空间分布（GASD）描述符
本文档介绍了全局对齐的空间分布描述符，可用于有效的对象识别和姿态估计
GASD基于对代表对象示例的整个点云的参考框架的估计，该参考框架用于将其与规范坐标系对齐。此后，根据对其的点云的3D点在空间上的分布方式，为它计算一个描述符。这样的描述符也可以在整个对准的
点云上扩展颜色分布，匹配点云的全局对齐变换用于计算对象姿态。

理论基础：
全局对齐的空间分布(GASD)全局描述方法将代表给定对象局部视图的3D点云作为输入。
（1）估计点云的参考系，这允许计算将其与规范坐标系对齐的变换，从而使描述符的姿势不变，
（2）对齐后，根据3D点的空间分布为点云计算形状描述符。
为了获得具有较高判别力的形状和颜色描述符，还可以考虑沿点云的颜色分布。然后通过匹配部分视图的查询和训练描述符来执行对象识别。还从匹配查询和训练局部视图的对齐转换中计算每个识别出的对象的姿态。
参考框架是使用主成分分析（PCA）方法估算的。给定Pi(i=1...n)代表对象局部代表对象局部视图的3D点集
（1）第一步是计算其质心--P--，--P--是参考帧的原点。
（2）然后，通过Pi和--P--来计算协方差矩阵C，见（https://pcl.readthedocs.io/projects/tutorials/en/latest/gasd_estimation.html#gasd-estimation 公式1）
            即将P内的每个点都与--P--相减，然后取平方和，再取平均值
（3）此后，C的特征值lemda i 和对应的特征向量Vj得到，具有j=1,2,3可以使得CVj=lemda j Vj。考虑到特征值以升序排列，因此将v1与最小特征值相关联的特征向量用作z参考帧的轴。如果V1与观看方向的角度在
(-90,90)范围内，则V1否定，这样就可以确保z始终指向观察者。x参考帧的轴是V3与最大特征值关联的特征向量。该y轴由下式给出V1=V1 X V2;
根据参考系，可以计算[R|t]将其与规范坐标系对齐的变换.Pi然后使用[R|t]定义局部视图的所有点，定义见（https://pcl.readthedocs.io/projects/tutorials/en/latest/gasd_estimation.html#gasd-estimation 矩阵1）
一旦使用框架对齐了点云，就可以从中计算出姿态不变的全局形状描述符。以原点为中心的点云轴对齐的边界立方体被分为ms x ms x ms规则网格。对于每个网格单元，将ls计算带有条带的直方图。如果为ls=1,则
每个直方图箱将在3D规则网格中存储属于其对应像元的点数。如果ls>1,则将为每个单元格计算每个样本与点云质心之间的标准化距离的直方图。
每个样本对直方图的贡献相对于点云中的点总数进行了归一化。可选的，可以使用插值法将每个样本的值分配到相邻的单元格中，以尝试避免边界效应（当样本从一个单元格内移动到另一个单元格时，边界效应可能
导致直方图的突然变化）。然后通过串联计算的直方图获得描述符。
见（https://pcl.readthedocs.io/projects/tutorials/en/latest/gasd_estimation.html#gasd-estimation 图1）
颜色信息也可以合并到描述符中以增加其区分能力。描述符的颜色分量是使用mc x mc x mc类似于形状分量的网格来计算的，但是会根据属于它的点的颜色为每个单元格生成颜色直方图。点云颜色表示在HSV空间中，
并且色相值累计在带有lc箱的直方图中 。与形状分量计算类似，执行关于点云数的归一化。另外，还可以执行直方图样本的内插。形状和颜色分量被连接起来，生成最终的描述符。
使用最邻近搜索方法来匹配查询和训练描述符。此后，对于每个匹配的对象实例，使用从相应查询和训练局部视图的参考帧获得的对齐变换来计算粗略姿势。给定变换[Rq|tq]并分量[Rt|tt]对齐查询和训练局部视图，则
对象粗略姿势[Rc|tc]可通过以下方式获得
见（https://pcl.readthedocs.io/projects/tutorials/en/latest/gasd_estimation.html#gasd-estimation 公式2）
[Rc|tc]然后可以使用迭代最近点(ICP)算法完善粗略姿势。

估算GASD功能
GASD在pcl中作为pcl_features库的一部分实现

彩色GASD参数的默认值为：Ms=6(一半为3),ls=1,Mc=4(一半为2)，lc=12并且没有直方图插值(INTERP_NONE)。这将产生984个浮点值的数组。它们存储在pcl::GASDSigature984点类型中。仅形状GASD参数的默认值为
Ms=8,ls=1和三线性直方图插值(INTERP_TRILINEAR)。这将产生一个512浮点值的数组，可以将其存储在pcl::GASDSignature512点类型中。也可以使用四边形直方图插值(INTERP_QUADRILINEAR)
*/

//估算输入色点云的GASD形状+颜色描述符
#include <pcl/point_types.h>
#include <pcl/features/gasd.h>

{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

  ... read, pass in or create a point cloud ...

  // Create the GASD estimation class, and pass the input dataset to it
  pcl::GASDColorEstimation<pcl::PointXYZRGBA, pcl::GASDSignature984> gasd;
  gasd.setInputCloud (cloud);

  // Output datasets
  pcl::PointCloud<pcl::GASDSignature984> descriptor;

  // Compute the descriptor
  gasd.compute (descriptor);

  // Get the alignment transform
  Eigen::Matrix4f trans = gasd.getTransform (trans);

  // Unpack histogram bins
  for (std::size_t i = 0; i < std::size_t( descriptor[0].descriptorSize ()); ++i)
  {
    descriptor[0].histogram[i];
  }
}
//估算输入点云的仅GASD形状描述符
#include <pcl/point_types.h>
#include <pcl/features/gasd.h>

{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  ... read, pass in or create a point cloud ...

  // Create the GASD estimation class, and pass the input dataset to it
  pcl::GASDEstimation<pcl::PointXYZ, pcl::GASDSignature512> gasd;
  gasd.setInputCloud (cloud);

  // Output datasets
  pcl::PointCloud<pcl::GASDSignature512> descriptor;

  // Compute the descriptor
  gasd.compute (descriptor);

  // Get the alignment transform
  Eigen::Matrix4f trans = gasd.getTransform (trans);

  // Unpack histogram bins
  for (std::size_t i = 0; i < std::size_t( descriptor[0].descriptorSize ()); ++i)
  {
    descriptor[0].histogram[i];
  }
}
