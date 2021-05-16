//
// Created by cs18 on 5/16/21.
//

/*基于惯性矩和偏心率的描述子
在本教程中，我们将学习如何使用pcl::MomentOfInetiaEstimation类来获取基于偏心率和惯性矩的描述符。该类还允许提取轴的对齐和定向的点云边界框。但是提起的OBB有可能并不是最小的边界框。

理论入门：
特征提取方法的思想如下：
（1）首先，计算点云的协方差矩阵，并提取其特征值和向量。我们可以认为所得到的特征向量已经归一化，并且始终形成右手坐标系（主要特征向量代表为X轴，次要特征向量代表为Z轴）。
（2）进行迭代过程，在每次迭代中，主要特征向量都旋转，旋转顺序始终是相同的，并且围绕其他特征向量执行，这为点云的旋转提供了不变性。今后，我们将旋转的主矢量称为当前轴。
见(https://pcl.readthedocs.io/projects/tutorials/en/latest/moment_of_inertia.html#moment-of-inertia  图1)
对于每个当前轴，都会计算出惯性矩。此外，当前轴也用于偏心率计算。因此，将当前矢量视为平面的法线矢量，并将 输入点云投影到该平面上。然后为获得的投影计算偏心率。
见(https://pcl.readthedocs.io/projects/tutorials/en/latest/moment_of_inertia.html#moment-of-inertia  图2)
实现的类还提供了获取AABB和OBB的方法。沿特征向量将定向边界框计算为AABB。



















*/

