////
//// Created by cs18 on 5/28/21.
////
///*基于对应分组的3D对象识别
//本教程旨在说明如何基于pcl_recognition模块执行3D对象识别。具体来说，他解释了如何使用对应分组算法，以将在3D描述符匹配阶段之后获得的点对点 对应关系集聚为当前场景中存在的模型实例。
//对于表示场景中可能的模型实例的每个群集，通讯分组算法还输出标识当前场景中该模型的6自由度姿态估计的变换矩阵。
//*/
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/correspondence.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/shot_omp.h>
//#include <pcl/features/board.h>
//#include <pcl/filters/uniform_sampling.h>
//#include <pcl/recognition/cg/hough_3d.h>
//#include <pcl/recognition/cg/geometric_consistency.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>
//#include <pcl/common/transforms.h>
//#include <pcl/console/parse.h>
//
//typedef pcl::PointXYZRGBA PointType;
//typedef pcl::Normal NormalType;
//typedef pcl::ReferenceFrame RFType;
//typedef pcl::SHOT352 DescriptorType;
//
//std::string model_filename_;
//std::string scene_filename_;
//
////Algorithm params
//bool show_keypoints_ (false);
//bool show_correspondences_ (false);
//bool use_cloud_resolution_ (false);
//bool use_hough_ (true);
//float model_ss_ (0.01f);
//float scene_ss_ (0.03f);
//float rf_rad_ (0.015f);
//float descr_rad_ (0.02f);
//float cg_size_ (0.01f);
//float cg_thresh_ (5.0f);
//
////打印出帮助信息
//void
//showHelp (char *filename)
//{
//  std::cout << std::endl;
//  std::cout << "***************************************************************************" << std::endl;
//  std::cout << "*                                                                         *" << std::endl;
//  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
//  std::cout << "*                                                                         *" << std::endl;
//  std::cout << "***************************************************************************" << std::endl << std::endl;
//  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
//  std::cout << "Options:" << std::endl;
//  std::cout << "     -h:                     Show this help." << std::endl;
//  std::cout << "     -k:                     Show used keypoints." << std::endl;
//  std::cout << "     -c:                     Show used correspondences." << std::endl;
//  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
//  std::cout << "                             each radius given by that value." << std::endl;
//  std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
//  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
//  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
//  std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
//  std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
//  std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
//  std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
//}
//
////解析命令行
//void
//parseCommandLine (int argc, char *argv[])
//{
//  //Show help
//  if (pcl::console::find_switch (argc, argv, "-h"))
//  {
//    showHelp (argv[0]);
//    exit (0);
//  }
//
//  //Model & scene filenames
//  std::vector<int> filenames;
//  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
//  if (filenames.size () != 2)
//  {
//    std::cout << "Filenames missing.\n";
//    showHelp (argv[0]);
//    exit (-1);
//  }
//
//  model_filename_ = argv[filenames[0]];
//  scene_filename_ = argv[filenames[1]];
//
//  //Program behavior
//  if (pcl::console::find_switch (argc, argv, "-k"))//将用于计算对应关系的关键点显示为PCL可视化器中的蓝色叠加层
//  {
//    show_keypoints_ = true;
//  }
//  if (pcl::console::find_switch (argc, argv, "-c"))//绘制一条线，将在聚类过程中保留下来的每对模型场景对应关系连接起来
//  {
//    show_correspondences_ = true;
//  }
//  if (pcl::console::find_switch (argc, argv, "-r"))//估计模型点云的空间分辨率，然后考虑用作参数的半径，就好像它们是以云分辨率为单位给出的一样；因此，实现了某种分辨率不变性，这在本教程与相同的命令行
//  //和不同的点云一起使用时可能会很有用
//  {
//    use_cloud_resolution_ = true;
//  }
//
//  std::string used_algorithm;
//  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
//  {
//    if (used_algorithm.compare ("Hough") == 0)//使用霍夫（Hough）聚类方法
//    {
//      use_hough_ = true;
//    }else if (used_algorithm.compare ("GC") == 0)//使用GC聚类方法
//    {
//      use_hough_ = false;
//    }
//    else
//    {
//      std::cout << "Wrong algorithm name.\n";
//      showHelp (argv[0]);
//      exit (-1);
//    }
//  }
//
//  //General parameters
//  pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
//  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
//  pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
//  pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
//  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
//  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
//}
//
////对给定点云进行空间分辨率计算以平均每个云点与其最临近点的距离
//double
//computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
//{
//  double res = 0.0;
//  int n_points = 0;
//  int nres;
//  std::vector<int> indices (2);
//  std::vector<float> sqr_distances (2);
//  pcl::search::KdTree<PointType> tree;
//  tree.setInputCloud (cloud);
//
//  for (std::size_t i = 0; i < cloud->size (); ++i)
//  {
//    if (! std::isfinite ((*cloud)[i].x))
//    {
//      continue;
//    }
//    //Considering the second neighbor since the first is the point itself.
//    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
//    if (nres == 2)
//    {
//      res += sqrt (sqr_distances[1]);
//      ++n_points;
//    }
//  }
//  if (n_points != 0)
//  {
//    res /= n_points;
//  }
//  return res;
//}
//
//int
//main (int argc, char *argv[])
//{
//  parseCommandLine (argc, argv);
//
//  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
//  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
//  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
//  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());
//
//  //
//  //  Load clouds，加载点云数据
//  //
//  if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
//  {
//    std::cout << "Error loading model cloud." << std::endl;
//    showHelp (argv[0]);
//    return (-1);
//  }
//  if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
//  {
//    std::cout << "Error loading scene cloud." << std::endl;
//    showHelp (argv[0]);
//    return (-1);
//  }
//
//  //
//  //  Set up resolution invariance，如果在命令行中启用了分辨率不变性标志，程序才会通过将其乘以估计的模型云分辨率来调整将在下一部分中使用的半径
//  //
//  if (use_cloud_resolution_)
//  {
//    float resolution = static_cast<float> (computeCloudResolution (model));
//    if (resolution != 0.0f)
//    {
//      model_ss_   *= resolution;
//      scene_ss_   *= resolution;
//      rf_rad_     *= resolution;
//      descr_rad_  *= resolution;
//      cg_size_    *= resolution;
//    }
//
//    std::cout << "Model resolution:       " << resolution << std::endl;
//    std::cout << "Model sampling size:    " << model_ss_ << std::endl;
//    std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
//    std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
//    std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
//    std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
//  }
//
//  //
//  //  Compute Normals，使用每个点最近的十个点来确定平面从而估计法线
//  //
//  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
//  norm_est.setKSearch (10);
//  norm_est.setInputCloud (model);
//  norm_est.compute (*model_normals);
//
//  norm_est.setInputCloud (scene);
//  norm_est.compute (*scene_normals);
//
//  //
//  //  Downsample Clouds to Extract keypoints，降采样点云获取关键点，然后将其与3D描述符关联以便执行关键点匹配并确定点对点的对应关系，UniformSampling的半径可以是命令行输入的半径，也可以是默认值
//  //
//
//  pcl::UniformSampling<PointType> uniform_sampling;
//  uniform_sampling.setInputCloud (model);
//  uniform_sampling.setRadiusSearch (model_ss_);
//  uniform_sampling.filter (*model_keypoints);
//  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;
//
//  uniform_sampling.setInputCloud (scene);
//  uniform_sampling.setRadiusSearch (scene_ss_);
//  uniform_sampling.filter (*scene_keypoints);
//  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;
//
//
//  //
//  //  Compute Descriptor for keypoints，将3D描述符与每个模型和场景关键点相关联，在这里我们使用SHOTEstimationOMP计算SHOT描述符。
//  //
//  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
//  descr_est.setRadiusSearch (descr_rad_);
//
//  descr_est.setInputCloud (model_keypoints);
//  descr_est.setInputNormals (model_normals);
//  descr_est.setSearchSurface (model);
//  descr_est.compute (*model_descriptors);
//
//  descr_est.setInputCloud (scene_keypoints);
//  descr_est.setInputNormals (scene_normals);
//  descr_est.setSearchSurface (scene);
//  descr_est.compute (*scene_descriptors);
//
//  //
//  //  Find Model-Scene Correspondences with KdTree，确定模型描述符和场景描述符之间的点对点的对应关系。我们使用KdTreeFLANN，输入设定为包含描述符的云。对于
//  //与场景关键点相关联的每个描述符，它会基于欧几里得距离有效地找到最相似的模型描述符，并将该对添加到对应向量CorrespondencesPtr中（仅当两个描述符是足够相似，即他们的平方距离小于阈值时）
//  //
//  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
//
//  pcl::KdTreeFLANN<DescriptorType> match_search;
//  match_search.setInputCloud (model_descriptors);
//
//  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
//  for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
//  {
//    std::vector<int> neigh_indices (1);
//    std::vector<float> neigh_sqr_dists (1);
//    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
//    {
//      continue;
//    }
//    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
//    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
//    {
//      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
//      model_scene_corrs->push_back (corr);
//    }
//  }
//  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
//
//  //
//  //  Actual Clustering，找到对应关系的实际聚类
//  //
//  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
//  std::vector<pcl::Correspondences> clustered_corrs;
//
//  //  Using Hough3D
//  if (use_hough_)
//  {
//    //
//    //  Compute (Keypoints) Reference Frames only for Hough，比较关键点的区别
//    //
//    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());//模型特征
//    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());//场景特征
//
//    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
//    rf_est.setFindHoles (true);
//    rf_est.setRadiusSearch (rf_rad_);//搜索半径
//
//    rf_est.setInputCloud (model_keypoints);//输入点云
//    rf_est.setInputNormals (model_normals);//输入法线
//    rf_est.setSearchSurface (model);//输入表面
//    rf_est.compute (*model_rf);//计算结果输出给model_rf
//
//    rf_est.setInputCloud (scene_keypoints);//设定输入点云
//    rf_est.setInputNormals (scene_normals);//设定法线
//    rf_est.setSearchSurface (scene);//设定输入表面
//    rf_est.compute (*scene_rf);//计算结果给到scene_rf
//
//    //  Clustering，聚合
//    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;//对象实例化
//    clusterer.setHoughBinSize (cg_size_);//设定哈希尺寸
//    clusterer.setHoughThreshold (cg_thresh_);
//    clusterer.setUseInterpolation (true);//设定是否使用插值
//    clusterer.setUseDistanceWeight (false);//设定是否使用距离权重
//
//    clusterer.setInputCloud (model_keypoints);//输入模型关键点
//    clusterer.setInputRf (model_rf);//输入模型识别特征点
//    clusterer.setSceneCloud (scene_keypoints);//输入场景关键点
//    clusterer.setSceneRf (scene_rf);//输入场景识别特征点
//    clusterer.setModelSceneCorrespondences (model_scene_corrs);//输入模型和场景对应分组
//
//    //clusterer.cluster (clustered_corrs);
//    clusterer.recognize (rototranslations, clustered_corrs);//设定重建参数，输出结果到clustered_corrs，该方法返回的向量分别为模型的每个实例的变换（旋转+平移）和
//  }
//  else // Using GeometricConsistency,使用GC分组
//  {
//    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
//    gc_clusterer.setGCSize (cg_size_);
//    gc_clusterer.setGCThreshold (cg_thresh_);
//
//    gc_clusterer.setInputCloud (model_keypoints);
//    gc_clusterer.setSceneCloud (scene_keypoints);
//    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);
//
//    //gc_clusterer.cluster (clustered_corrs);
//    gc_clusterer.recognize (rototranslations, clustered_corrs);
//  }
//
//  //
//  //  Output results，输出结果
//  //
//  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
//  for (std::size_t i = 0; i < rototranslations.size (); ++i)
//  {
//    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
//    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;
//
//    // Print the rotation matrix and translation vector
//    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
//    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
//
//    printf ("\n");
//    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
//    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
//    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
//    printf ("\n");
//    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
//  }
//
//  //
//  //  Visualization
//  //
//  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
//  viewer.addPointCloud (scene, "scene_cloud");
//
//  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());
//
//  if (show_correspondences_ || show_keypoints_)
//  {
//    //  We are translating the model so that it doesn't end in the middle of the scene representation
//    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
//    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
//
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
//    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
//  }
//
//  if (show_keypoints_)
//  {
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
//    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
//
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
//    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
//  }
//
//  for (std::size_t i = 0; i < rototranslations.size (); ++i)
//  {
//    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
//    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
//
//    std::stringstream ss_cloud;
//    ss_cloud << "instance" << i;
//
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
//    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());
//
//    if (show_correspondences_)
//    {
//      for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j)
//      {
//        std::stringstream ss_line;
//        ss_line << "correspondence_line" << i << "_" << j;
//        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
//        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);
//
//        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
//        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
//      }
//    }
//  }
//
//  while (!viewer.wasStopped ())
//  {
//    viewer.spinOnce ();
//  }
//
//  return (0);
//}
//
