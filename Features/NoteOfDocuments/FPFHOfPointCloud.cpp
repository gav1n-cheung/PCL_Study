//
// Created by cs18 on 5/15/21.
//

/*
快速点特征直方图（FPFH）描述符

点特征直方图的理论计算复杂度为O(nk^2)(给定的点云P，点数量n，k为P内的p的近邻点数量）。对于实时或近实时应用，密集点邻域中的点特征直方图的计算代表其瓶颈之一。

本教程描述了PFH公式的简化，称为快速点特征直方图(FPFH)，该算法将算法的计算复杂度降低至O(nk)，同时仍保留了PFH的大多数判别能力。

理论基础：
为了简化直方图特征的计算，我们进行如下操作：
    （1）对于每个查询点，如PFH所述，计算其自身及其邻居之间的Pq一组点元素，这称为简化点特征直方图(SPFH);\ alpha，\ phi，\ theta
    （2）对于每个点k，重新确定其近邻点，并且使用临近SPFH值加权最终的直方图Pq，这称为FPFH，如 （https://pcl.readthedocs.io/projects/tutorials/en/latest/fpfh_estimation.html#fpfh-estimation)_公式1所示
    其中权重wi表示在某个给定度量空间中查询点Pq和邻居点之间的距离Pi，从而对(Pq,Pi)进行了评分，但也可以根据需要选择作为其他度量，为了理解次加权方案的重要性，下图展示了以Pq为主的k邻域集的影响区域图。
这里我们对PFH和FPFH做一下区分。
其中，PFH是对邻域球体内所有对点（k个）之间的关系的直方图，而FPFH则是-------首先，对点云内每个点进行SPFH计算；然后，Pq其邻居点(k个)的的SPFH进行重新加权；这样Pq本身的SPFH和Pk的SPFH加权就得到了
Pq的FPFH。很容易理解的，原来我们求解目标点的PFH，需要对k相邻点都做这样的操作，这样对键值对的操作，使得计算量为O(k^2)。而求取FPFH其不需要单独为一个目标点做额外的求取对操作，我们得到了所有点的
SPFH，对其进行加权就可以得到FPFH，这样的话，计算复杂度为O(k)。大大减少了计算量。
见（https://pcl.readthedocs.io/projects/tutorials/en/latest/fpfh_estimation.html#fpfh-estimation 图1）
其中，红色线是计算Pq的FPFH所需的k临近点对，而黑色线则是Pk点计算FPFH所需的点对。其中某些值对需要被计数两次(图中用粗线标记--红黑色线皆是)

PFH和FPFH的区别：见（https://pcl.readthedocs.io/projects/tutorials/en/latest/fpfh_estimation.html#fpfh-estimation 图2）
（1）Pq从图中可以得到，FPFH并没有完全互联其所有的相邻点，因此缺少了一些值对，这些值可能有助于捕获查询点周围的几何图像。
（2）PFH在查询点周围对精确确定的表面进行建模，而FPFH在r半径之外(最多为2r处)包括了其他点对
（3）由于采用了重新加权方案，FPFH合并了SPFH值并重新获取了一些点相邻值对。
（4）FPFH的整体复杂性大大下降，因此可以在实时应用中使用
（5）通过对值进行去相关，可以简化生成的之凡图，即仅创建d个单独的特征直方图（每个特征维一个），并将它们连接在一起

估计PFH功能：
 FPFHEstimation类的实际计算调用在内部不执行任何操作，但：
for each point p in cloud P  //遍历P内的每个点p

  1. pass 1:

     1. get the nearest neighbors of :math:`p`      //获取到p的最临近点

     2. for each pair of :math:`p, p_i` (where :math:`p_i` is a neighbor of :math:`p`, compute the three angular values   //对于每个p的每个临近点组成点值对，计算其三个角度

     3. bin all the results in an output SPFH histogram //将所有结果给到输出

  2. pass 2:

     1. get the nearest neighbors of :math:`p` //获取到p的最临近点

     3. use each SPFH of :math:`p` with a weighting scheme to assemble the FPFH of :math:`p`: //使用p的每个SPFH加权得到p的FPFH

同样的，我们在计算SPFH之前也要对发现进行检测，在此不再赘述


































*/
