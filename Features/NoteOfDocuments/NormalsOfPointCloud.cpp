//
// Created by cs18 on 5/12/21.
//
/*
估计PointCloud中的表面法线
 表面法线是几何表面的重要属性，在许多领域（例如计算机图形应用程序）中大量使用，以应用正确的光源以产生阴影和其他视觉效果。
 给定一个几何表面，通常推论该表面上某个点的法线方向为垂直于该点表面的向量是很容易的。但是，由于我们获取的点云数据集表示真实表面上的一组点样本，因此有两种可能性：
 （1）使用表面网格化技术从获取的点云数据集中获取基础表面，然后从网格中计算表面法线
 （2）使用近似值直接从点云数据集中推断出表面法线
 本节将解决后者，即给定点云数据集，直接计算点云中每个点的表面法线。

 基础理论
 正态估计的方法有很多种，但我们集中于本节的方法是最简单的方法之一，其公式如（https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html#normal-estimation 公式1）。
 确定表面上某个点的法线的问题可以通过估计与该表面相切的平面的法线的问题来近似，这又成为了一个最小二乘平面拟合估计问题。
 因此，用于估计表面法线的解决方案简化为对从查询点的最近临近点创建的协方差矩阵的特征向量和特征值进行分析。更具体地说，对于每个点Pi，我们组织协方差矩阵C如上公式所示。
 其中，k为Pi临近点的数目，-p-表示3D质心最近的临近点，ambda_j是协方差矩阵的第j个特征值，并且Vj->存在j个特征向量。
 要从PCL中的一组点估计协方差矩阵，可以使用

  // Placeholder for the 3x3 covariance matrix at each surface patch。定义每个贴面的3X3协方差矩阵
  Eigen::Matrix3f covariance_matrix;
  // 16-bytes aligned placeholder for the XYZ centroid of a surface patch。定义16字节对齐的表面补丁的XYZ中心点的占位符。
  Eigen::Vector4f xyz_centroid;

  // Estimate the XYZ centroid。估计XYZ的中心点
  compute3DCentroid (cloud, xyz_centroid);

  // Compute the 3x3 covariance matrix。计算3X3协方差矩阵
  computeCovarianceMatrix (cloud, xyz_centroid, covariance_matrix);

通常，由于没有数学方法可以解决法线的正负号问题，因此如上所示，通过主成分分析（PCA）计算出的法线方向是模棱两可的，并且在整个点云数据集上的方向不一致。
由于数据集是2.5D的，因此是从单个角度获取的，因此法线应该仅出现在模型的一半上。但是由于方向不一致，他们分布于整个球体上。
而如果观点Vp实际上是已知的，那解决该问题是很简单的。为了使所有法线Ni-->始终一致地朝向视点，它们需要满足以下方程式：Ni-->*(Vp-Pi)>0
要在PCL中手动重新定向给定法线，可以使用
flipNormalTowardsViewpoint (const PointT &point, float vp_x, float vp_y, float vp_z, Eigen::Vector4f &normal);
        如果数据集具有多个采集视点，则上述常规重定位方法将不成立，并且需要实现更复杂的算法。

选择正确的比例
如前所述，需要根据该点的周围点邻域支持（也称为k邻域）估算该点的表面法线。
最近邻估计的细节提出了正确比例因子的问题：给定采样点云数据集，正确的k值（通过pcl::Feature::setKSearch给出）或r值（通过pcl::Feature::setRadiusSearch给出）应该是什么？这样的值才能用于确定的
最近邻近点集。
这个问题是非常重要的，并且构成了点特征表示的自动估计（即没有手动给定阈值的情况下）的限制因素。为了更好地说明此问题，我们用两个例子来对比这个值的影响。
见（https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html#normal-estimation  法线对比图）。
图中显示了选择较小比例（即r或k值较小）与较大比例（即r或k值较大）的效果。左图描绘了一个合理选择的比例因子，其中估计的表面法线大致垂直于两个平面，在下面的图片中可以看到桌子上的较小细节。
但如果设置较大的k或r值，则相邻对象的集合会覆盖来自相邻曲面的较大点，则估计的点要素表示会失真，在两个平面边缘处旋转的曲面法线会被涂抹边缘并且压制精细细节。
无需过多讨论，就足以假设目前，必须根据应用程序所需的详细晨读来选择确定点的邻域的比例。简而言之，如果杯子的手柄和圆柱部分之间的边缘处的曲率很重要，则比例因子必须足够小以捕获这些细节，否则
比例因子要大。
执行main.cpp中的代码可以看到预设数据集的表面法线预估结果。

来自NormalEstimation类的实际计算调用在内部不执行任何操作，但其有以下作用：
    for each point p in cloud P                 //遍历点云P中的每个点p

  1. get the nearest neighbors of p         //获取p的最临近点

  2. compute the surface normal n of p    //计算p的平面法线n

  3. check if n is consistently oriented towards the viewpoint and flip otherwise   //检查n是否始终朝向视点，否则就翻转n

视点默认设置为(0,0,0)，可以通过以下方法更改视点：
    setViewPoint (float vpx, float vpy, float vpz);

如果要计算单点法线，则可以使用：
    computePointNormal (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices, Eigen::Vector4f &plane_parameters, float &curvature);
    其中cloud是我们要计算的点所属的输入点云，indices表示点云中k个近邻点的集合，plane_parameters和curvature表示法线估计的输出。plane_parameters保持
    法线前三个坐标（nx,ny,nz）不变，第四个坐标是D=nc.p_plane(此处为质心)+p。输出表面曲率被估计为协方差矩阵特征值之间的关系，
    如https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html#normal-estimation 公式2所示。


 */