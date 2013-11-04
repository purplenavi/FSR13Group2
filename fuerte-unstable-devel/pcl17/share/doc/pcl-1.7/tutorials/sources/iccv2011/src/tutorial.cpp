#include <vector>
#include <string>
#include <sstream>
#include <pcl17/io/pcd_io.h>
#include <pcl17/registration/transforms.h>
#include <pcl17/visualization/pcl_visualizer.h>
#include <pcl17/keypoints/sift_keypoint.h>

#include <pcl17/pcl_config.h>
#if PCL17_MAJOR_VERSION >= 1 && PCL17_MINOR_VERSION >= 7
#  include <pcl17/keypoints/harris_3d.h>
#else
#  include <pcl17/keypoints/harris_3d.h>
#endif

#include <pcl17/ModelCoefficients.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/search/kdtree.h>
#include <pcl17/segmentation/extract_clusters.h>
#include <pcl17/features/fpfh_omp.h>
#include <pcl17/features/pfh.h>
#include <pcl17/features/pfhrgb.h>
#include <pcl17/features/3dsc.h>
#include <pcl17/features/shot_omp.h>
#include <pcl17/kdtree/kdtree_flann.h>
#include <pcl17/kdtree/impl/kdtree_flann.hpp>
#include <pcl17/registration/transformation_estimation_svd.h>
#include <pcl17/registration/icp.h>
#include <pcl17/registration/correspondence_rejection_sample_consensus.h>
#include <pcl17/common/transforms.h>
#include <pcl17/surface/grid_projection.h>
#include <pcl17/surface/gp3.h>
#include <pcl17/surface/marching_cubes_hoppe.h>

template<typename FeatureType>
class ICCVTutorial
{
  public:
    ICCVTutorial (boost::shared_ptr<pcl17::Keypoint<pcl17::PointXYZRGB, pcl17::PointXYZI> > keypoint_detector,
                  typename pcl17::Feature<pcl17::PointXYZRGB, FeatureType>::Ptr feature_extractor,
                  boost::shared_ptr<pcl17::PCLSurfaceBase<pcl17::PointXYZRGBNormal> > surface_reconstructor,
                  typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr source,
                  typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr target);
    
    /**
     * @brief starts the event loop for the visualizer
     */
    void run ();
  protected:
    /**
     * @brief remove plane and select largest cluster as input object
     * @param input the input point cloud
     * @param segmented the resulting segmented point cloud containing only points of the largest cluster
     */
    void segmentation (typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr input, typename pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr segmented) const;
    
    /**
     * @brief Detects key points in the input point cloud
     * @param input the input point cloud
     * @param keypoints the resulting key points. Note that they are not necessarily a subset of the input cloud
     */
    void detectKeypoints (typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr input, pcl17::PointCloud<pcl17::PointXYZI>::Ptr keypoints) const;
    
    /**
     * @brief extract descriptors for given key points
     * @param input point cloud to be used for descriptor extraction
     * @param keypoints locations where descriptors are to be extracted
     * @param features resulting descriptors
     */
    void extractDescriptors (typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr input, typename pcl17::PointCloud<pcl17::PointXYZI>::Ptr keypoints, typename pcl17::PointCloud<FeatureType>::Ptr features);
    
    /**
     * @brief find corresponding features based on some metric
     * @param source source feature descriptors
     * @param target target feature descriptors 
     * @param correspondences indices out of the target descriptors that correspond (nearest neighbor) to the source descriptors
     */    
    void findCorrespondences (typename pcl17::PointCloud<FeatureType>::Ptr source, typename pcl17::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const;
    
    /**
     * @brief  remove non-consistent correspondences
     */
    void filterCorrespondences ();
    
    /**
     * @brief calculate the initial rigid transformation from filtered corresponding keypoints
     */
    void determineInitialTransformation ();
    
    /**
     * @brief calculate the final rigid transformation using ICP over all points
     */
    void determineFinalTransformation ();

    /**
     * @brief reconstructs the surface from merged point clouds
     */
    void reconstructSurface ();

    /**
     * @brief callback to handle keyboard events
     * @param event object containing information about the event. e.g. type (press, release) etc.
     * @param cookie user defined data passed during registration of the callback
     */
    void keyboard_callback (const pcl17::visualization::KeyboardEvent& event, void* cookie);
    
  private:
    pcl17::visualization::PCLVisualizer visualizer_;
    pcl17::PointCloud<pcl17::PointXYZI>::Ptr source_keypoints_;
    pcl17::PointCloud<pcl17::PointXYZI>::Ptr target_keypoints_;
    boost::shared_ptr<pcl17::Keypoint<pcl17::PointXYZRGB, pcl17::PointXYZI> > keypoint_detector_;
    typename pcl17::Feature<pcl17::PointXYZRGB, FeatureType>::Ptr feature_extractor_;
    boost::shared_ptr<pcl17::PCLSurfaceBase<pcl17::PointXYZRGBNormal> > surface_reconstructor_;
    typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr source_;
    typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr target_;
    typename pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr source_segmented_;
    typename pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr target_segmented_;
    typename pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr source_transformed_;
    typename pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr source_registered_;
    typename pcl17::PolygonMesh surface_;
    typename pcl17::PointCloud<FeatureType>::Ptr source_features_;
    typename pcl17::PointCloud<FeatureType>::Ptr target_features_;
    std::vector<int> source2target_;
    std::vector<int> target2source_;
    pcl17::CorrespondencesPtr correspondences_;
    Eigen::Matrix4f initial_transformation_matrix_;
    Eigen::Matrix4f transformation_matrix_;
    bool show_source2target_;
    bool show_target2source_;
    bool show_correspondences;
};

template<typename FeatureType>
ICCVTutorial<FeatureType>::ICCVTutorial(boost::shared_ptr<pcl17::Keypoint<pcl17::PointXYZRGB, pcl17::PointXYZI> >keypoint_detector,
                                        typename pcl17::Feature<pcl17::PointXYZRGB, FeatureType>::Ptr feature_extractor,
                                        boost::shared_ptr<pcl17::PCLSurfaceBase<pcl17::PointXYZRGBNormal> > surface_reconstructor,
                                        typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr source,
                                        typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr target)
: source_keypoints_ (new pcl17::PointCloud<pcl17::PointXYZI> ())
, target_keypoints_ (new pcl17::PointCloud<pcl17::PointXYZI> ())
, keypoint_detector_ (keypoint_detector)
, feature_extractor_ (feature_extractor)
, surface_reconstructor_ (surface_reconstructor)
, source_ (source)
, target_ (target)
, source_segmented_ (new pcl17::PointCloud<pcl17::PointXYZRGB>)
, target_segmented_ (new pcl17::PointCloud<pcl17::PointXYZRGB>)
, source_transformed_ (new pcl17::PointCloud<pcl17::PointXYZRGB>)
, source_registered_ (new pcl17::PointCloud<pcl17::PointXYZRGB>)
, source_features_ (new pcl17::PointCloud<FeatureType>)
, target_features_ (new pcl17::PointCloud<FeatureType>)
, correspondences_ (new pcl17::Correspondences)
, show_source2target_ (false)
, show_target2source_ (false)
, show_correspondences (false)
{
  visualizer_.registerKeyboardCallback(&ICCVTutorial::keyboard_callback, *this, 0);
  
  segmentation (source_, source_segmented_);
  segmentation (target_, target_segmented_);  
  
  detectKeypoints (source_segmented_, source_keypoints_);
  detectKeypoints (target_segmented_, target_keypoints_);
  
  extractDescriptors (source_segmented_, source_keypoints_, source_features_);
  extractDescriptors (target_segmented_, target_keypoints_, target_features_);
  
  findCorrespondences (source_features_, target_features_, source2target_);
  findCorrespondences (target_features_, source_features_, target2source_);
  
  filterCorrespondences ();
  
  determineInitialTransformation ();
  determineFinalTransformation ();
  
  reconstructSurface ();
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::segmentation (typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr source, typename pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr segmented) const
{
  cout << "segmentation..." << std::flush;
  // fit plane and keep points above that plane
  pcl17::ModelCoefficients::Ptr coefficients (new pcl17::ModelCoefficients);
  pcl17::PointIndices::Ptr inliers (new pcl17::PointIndices);
  // Create the segmentation object
  pcl17::SACSegmentation<pcl17::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl17::SACMODEL_PLANE);
  seg.setMethodType (pcl17::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);

  seg.setInputCloud (source);
  seg.segment (*inliers, *coefficients);
  
  pcl17::ExtractIndices<pcl17::PointXYZRGB> extract;
  extract.setInputCloud (source);
  extract.setIndices (inliers);
  extract.setNegative (true);

  extract.filter (*segmented);
  std::vector<int> indices;
  pcl17::removeNaNFromPointCloud(*segmented, *segmented, indices);
  cout << "OK" << endl;
  
  cout << "clustering..." << std::flush;
  // euclidean clustering
  typename pcl17::search::KdTree<pcl17::PointXYZRGB>::Ptr tree (new pcl17::search::KdTree<pcl17::PointXYZRGB>);
  tree->setInputCloud (segmented);

  std::vector<pcl17::PointIndices> cluster_indices;
  pcl17::EuclideanClusterExtraction<pcl17::PointXYZRGB> clustering;
  clustering.setClusterTolerance (0.02); // 2cm
  clustering.setMinClusterSize (1000);
  clustering.setMaxClusterSize (250000);
  clustering.setSearchMethod (tree);
  clustering.setInputCloud(segmented);
  clustering.extract (cluster_indices);
  
  if (cluster_indices.size() > 0)//use largest cluster
  {
    cout << cluster_indices.size() << " clusters found";
    if (cluster_indices.size() > 1)
      cout <<" Using largest one...";
    cout << endl;
    typename pcl17::IndicesPtr indices (new std::vector<int>);
    *indices = cluster_indices[0].indices;
    extract.setInputCloud (segmented);
    extract.setIndices (indices);
    extract.setNegative (false);

    extract.filter (*segmented);
  }
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::detectKeypoints (typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr input, pcl17::PointCloud<pcl17::PointXYZI>::Ptr keypoints) const
{
  cout << "keypoint detection..." << std::flush;
  keypoint_detector_->setInputCloud(input);
  keypoint_detector_->setSearchSurface(input);
  keypoint_detector_->compute(*keypoints);
  cout << "OK. keypoints found: " << keypoints->points.size() << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::extractDescriptors (typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr input, typename pcl17::PointCloud<pcl17::PointXYZI>::Ptr keypoints, typename pcl17::PointCloud<FeatureType>::Ptr features)
{
  typename pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr kpts(new pcl17::PointCloud<pcl17::PointXYZRGB>);
  kpts->points.resize(keypoints->points.size());
  
  pcl17::copyPointCloud(*keypoints, *kpts);
          
  typename pcl17::FeatureFromNormals<pcl17::PointXYZRGB, pcl17::Normal, FeatureType>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl17::FeatureFromNormals<pcl17::PointXYZRGB, pcl17::Normal, FeatureType> > (feature_extractor_);
  
  feature_extractor_->setSearchSurface(input);
  feature_extractor_->setInputCloud(kpts);
  
  if (feature_from_normals)
  //if (boost::dynamic_pointer_cast<typename pcl17::FeatureFromNormals<pcl17::PointXYZRGB, pcl17::Normal, FeatureType> > (feature_extractor_))
  {
    cout << "normal estimation..." << std::flush;
    typename pcl17::PointCloud<pcl17::Normal>::Ptr normals (new  pcl17::PointCloud<pcl17::Normal>);
    pcl17::NormalEstimation<pcl17::PointXYZRGB, pcl17::Normal> normal_estimation;
    normal_estimation.setSearchMethod (pcl17::search::Search<pcl17::PointXYZRGB>::Ptr (new pcl17::search::KdTree<pcl17::PointXYZRGB>));
    normal_estimation.setRadiusSearch (0.01);
    normal_estimation.setInputCloud (input);
    normal_estimation.compute (*normals);
    feature_from_normals->setInputNormals(normals);
    cout << "OK" << endl;
  }

  cout << "descriptor extraction..." << std::flush;
  feature_extractor_->compute (*features);
  cout << "OK" << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::findCorrespondences (typename pcl17::PointCloud<FeatureType>::Ptr source, typename pcl17::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const
{
  cout << "correspondence assignment..." << std::flush;
  correspondences.resize (source->size());

  // Use a KdTree to search for the nearest matches in feature space
  pcl17::KdTreeFLANN<FeatureType> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target);

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source->size (); ++i)
  {
    descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
    correspondences[i] = k_indices[0];
  }
  cout << "OK" << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::filterCorrespondences ()
{
  cout << "correspondence rejection..." << std::flush;
  std::vector<std::pair<unsigned, unsigned> > correspondences;
  for (unsigned cIdx = 0; cIdx < source2target_.size (); ++cIdx)
    if (target2source_[source2target_[cIdx]] == cIdx)
      correspondences.push_back(std::make_pair(cIdx, source2target_[cIdx]));
  
  correspondences_->resize (correspondences.size());
  for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
  {
    (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
    (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
  }
  
  pcl17::registration::CorrespondenceRejectorSampleConsensus<pcl17::PointXYZI> rejector;
  rejector.setInputCloud(source_keypoints_);
  rejector.setTargetCloud(target_keypoints_);
  rejector.setInputCorrespondences(correspondences_);
  rejector.getCorrespondences(*correspondences_);
  cout << "OK" << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::determineInitialTransformation ()
{
  cout << "initial alignment..." << std::flush;
  pcl17::registration::TransformationEstimation<pcl17::PointXYZI, pcl17::PointXYZI>::Ptr transformation_estimation (new pcl17::registration::TransformationEstimationSVD<pcl17::PointXYZI, pcl17::PointXYZI>);
  
  transformation_estimation->estimateRigidTransformation (*source_keypoints_, *target_keypoints_, *correspondences_, initial_transformation_matrix_);
  
  pcl17::transformPointCloud(*source_segmented_, *source_transformed_, initial_transformation_matrix_);
  cout << "OK" << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::determineFinalTransformation ()
{
  cout << "final registration..." << std::flush;
  pcl17::Registration<pcl17::PointXYZRGB, pcl17::PointXYZRGB>::Ptr registration (new pcl17::IterativeClosestPoint<pcl17::PointXYZRGB, pcl17::PointXYZRGB>);
  registration->setInputCloud(source_transformed_);
  //registration->setInputCloud(source_segmented_);
  registration->setInputTarget (target_segmented_);
  registration->setMaxCorrespondenceDistance(0.05);
  registration->setRANSACOutlierRejectionThreshold (0.05);
  registration->setTransformationEpsilon (0.000001);
  registration->setMaximumIterations (1000);
  registration->align(*source_registered_);
  transformation_matrix_ = registration->getFinalTransformation();
  cout << "OK" << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::reconstructSurface ()
{
  cout << "surface reconstruction..." << std::flush;
  // merge the transformed and the target point cloud
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr merged (new pcl17::PointCloud<pcl17::PointXYZRGB>);
  *merged = *source_transformed_;
  *merged += *target_segmented_;
  
  // apply grid filtering to reduce amount of points as well as to make them uniform distributed
  pcl17::VoxelGrid<pcl17::PointXYZRGB> voxel_grid;
  voxel_grid.setInputCloud(merged);
  voxel_grid.setLeafSize(0.002, 0.002, 0.002);
  voxel_grid.setDownsampleAllData(true);
  voxel_grid.filter(*merged);

  pcl17::PointCloud<pcl17::PointXYZRGBNormal>::Ptr vertices (new pcl17::PointCloud<pcl17::PointXYZRGBNormal>);
  pcl17::copyPointCloud(*merged, *vertices);

  pcl17::NormalEstimation<pcl17::PointXYZRGB, pcl17::PointXYZRGBNormal> normal_estimation;
  normal_estimation.setSearchMethod (pcl17::search::Search<pcl17::PointXYZRGB>::Ptr (new pcl17::search::KdTree<pcl17::PointXYZRGB>));
  normal_estimation.setRadiusSearch (0.01);
  normal_estimation.setInputCloud (merged);
  normal_estimation.compute (*vertices);
  
  pcl17::search::KdTree<pcl17::PointXYZRGBNormal>::Ptr tree (new pcl17::search::KdTree<pcl17::PointXYZRGBNormal>);
  tree->setInputCloud (vertices);

  surface_reconstructor_->setSearchMethod(tree);
  surface_reconstructor_->setInputCloud(vertices);
  surface_reconstructor_->reconstruct(surface_);
  cout << "OK" << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::run()
{
  visualizer_.spin ();
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::keyboard_callback (const pcl17::visualization::KeyboardEvent& event, void* cookie)
{
  if (event.keyUp())
  {
    switch (event.getKeyCode())
    {
      case '1':
        if (!visualizer_.removePointCloud("source_points"))
        {
          visualizer_.addPointCloud(source_, "source_points");
        }
        break;
        
      case '2':
        if (!visualizer_.removePointCloud("target_points"))
        {
          visualizer_.addPointCloud(target_, "target_points");
        }
        break;
      
      case '3':
        if (!visualizer_.removePointCloud("source_segmented"))
        {
          visualizer_.addPointCloud(source_segmented_, "source_segmented");
        }
        break;
        
      case '4':
        if (!visualizer_.removePointCloud("target_segmented"))
        {
          visualizer_.addPointCloud(target_segmented_, "target_segmented");
        }
        break;
        
      case '5':
        if (!visualizer_.removePointCloud("source_keypoints"))
        {
          pcl17::visualization::PointCloudColorHandlerCustom<pcl17::PointXYZI> keypoint_color (source_keypoints_, 0, 0, 255);
          //pcl17::visualization::PointCloudColorHandlerGenericField<pcl17::PointXYZI> keypoint_color (source_keypoints_, "intensity");
          visualizer_.addPointCloud(source_keypoints_, keypoint_color, "source_keypoints");
        }
        break;
      
      case '6':
        if (!visualizer_.removePointCloud("target_keypoints"))
        {
          //pcl17::visualization::PointCloudColorHandlerGenericField<pcl17::PointXYZI> keypoint_color (target_keypoints_, "intensity");
          pcl17::visualization::PointCloudColorHandlerCustom<pcl17::PointXYZI> keypoint_color (target_keypoints_, 255, 0, 0);
          visualizer_.addPointCloud(target_keypoints_, keypoint_color, "target_keypoints");
        }
        break;

      case '7':
        if (!show_source2target_)
          visualizer_.addCorrespondences<pcl17::PointXYZI>(source_keypoints_, target_keypoints_, source2target_, "source2target");
        else
          visualizer_.removeCorrespondences("source2target");
          
        show_source2target_ = !show_source2target_;
        break;

      case '8':
        if (!show_target2source_)
          visualizer_.addCorrespondences<pcl17::PointXYZI>(target_keypoints_, source_keypoints_, target2source_, "target2source");
        else
          visualizer_.removeCorrespondences("target2source");

        show_target2source_ = !show_target2source_;
        break;
        
      case '9':
        if (!show_correspondences)
          visualizer_.addCorrespondences<pcl17::PointXYZI>(source_keypoints_, target_keypoints_, *correspondences_, "correspondences");
        else
          visualizer_.removeCorrespondences("correspondences");
        show_correspondences = !show_correspondences;
        break;
        
      case 'i':
      case 'I':
        if (!visualizer_.removePointCloud("transformed"))
          visualizer_.addPointCloud(source_transformed_, "transformed");
        break;

      case 'r':
      case 'R':
        if (!visualizer_.removePointCloud("registered"))
          visualizer_.addPointCloud(source_registered_, "registered");
        break;
        
      case 't':
      case 'T':
          visualizer_.addPolygonMesh(surface_, "surface");
        break;
    }
  }
}

int 
main (int argc, char ** argv)
{
  if (argc < 6) 
  {
    pcl17::console::print_info ("Syntax is: %s <source-pcd-file> <target-pcd-file> <keypoint-method> <descriptor-type> <surface-reconstruction-method>\n", argv[0]);
    pcl17::console::print_info ("available <keypoint-methods>: 1 = Sift3D\n");
    pcl17::console::print_info ("                              2 = Harris3D\n");
    pcl17::console::print_info ("                              3 = Tomasi3D\n");
    pcl17::console::print_info ("                              4 = Noble3D\n");
    pcl17::console::print_info ("                              5 = Lowe3D\n");
    pcl17::console::print_info ("                              6 = Curvature3D\n\n");
    pcl17::console::print_info ("available <descriptor-types>: 1 = FPFH\n");
    pcl17::console::print_info ("                              2 = SHOTRGB\n");
    pcl17::console::print_info ("                              3 = PFH\n");
    pcl17::console::print_info ("                              4 = PFHRGB\n\n");
    pcl17::console::print_info ("available <surface-methods>:  1 = Greedy Projection\n");
    pcl17::console::print_info ("                              2 = Marching Cubes\n");    
    
    return (1);
  }
  pcl17::console::print_info ("== MENU ==\n");
  pcl17::console::print_info ("1 - show/hide source point cloud\n");
  pcl17::console::print_info ("2 - show/hide target point cloud\n");
  pcl17::console::print_info ("3 - show/hide segmented source point cloud\n");
  pcl17::console::print_info ("4 - show/hide segmented target point cloud\n");
  pcl17::console::print_info ("5 - show/hide source key points\n");
  pcl17::console::print_info ("6 - show/hide target key points\n");
  pcl17::console::print_info ("7 - show/hide source2target correspondences\n");
  pcl17::console::print_info ("8 - show/hide target2source correspondences\n");
  pcl17::console::print_info ("9 - show/hide consistent correspondences\n");
  pcl17::console::print_info ("i - show/hide initial alignment\n");
  pcl17::console::print_info ("r - show/hide final registration\n");
  pcl17::console::print_info ("t - show/hide triangulation (surface reconstruction)\n");
  pcl17::console::print_info ("h - show visualizer options\n");
  pcl17::console::print_info ("q - quit\n");
  
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr source (new pcl17::PointCloud<pcl17::PointXYZRGB>);
  pcl17::io::loadPCDFile (argv[1], *source);
  
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr target (new pcl17::PointCloud<pcl17::PointXYZRGB>);
  pcl17::io::loadPCDFile (argv[2], *target);
  
  int keypoint_type   = atoi (argv[3]);
  int descriptor_type = atoi (argv[4]);
  int surface_type    = atoi (argv[5]);
  
  boost::shared_ptr<pcl17::Keypoint<pcl17::PointXYZRGB, pcl17::PointXYZI> > keypoint_detector;
  
  if (keypoint_type == 1)
  {
    pcl17::SIFTKeypoint<pcl17::PointXYZRGB, pcl17::PointXYZI>* sift3D = new pcl17::SIFTKeypoint<pcl17::PointXYZRGB, pcl17::PointXYZI>;
    sift3D->setScales(0.01, 3, 2);
    sift3D->setMinimumContrast(0.0);
    keypoint_detector.reset(sift3D);
  }
  else
  {
    pcl17::HarrisKeypoint3D<pcl17::PointXYZRGB,pcl17::PointXYZI>* harris3D = new pcl17::HarrisKeypoint3D<pcl17::PointXYZRGB,pcl17::PointXYZI> (pcl17::HarrisKeypoint3D<pcl17::PointXYZRGB,pcl17::PointXYZI>::HARRIS);
    harris3D->setNonMaxSupression(true);
    harris3D->setRadius (0.01);
    harris3D->setRadiusSearch (0.01);
    keypoint_detector.reset(harris3D);
    switch (keypoint_type)
    {
      case 2:
        harris3D->setMethod(pcl17::HarrisKeypoint3D<pcl17::PointXYZRGB,pcl17::PointXYZI>::HARRIS);
      break;

      case 3:
        harris3D->setMethod(pcl17::HarrisKeypoint3D<pcl17::PointXYZRGB,pcl17::PointXYZI>::TOMASI);
      break;

      case 4:
        harris3D->setMethod(pcl17::HarrisKeypoint3D<pcl17::PointXYZRGB,pcl17::PointXYZI>::NOBLE);
      break;
      
      case 5:
        harris3D->setMethod(pcl17::HarrisKeypoint3D<pcl17::PointXYZRGB,pcl17::PointXYZI>::LOWE);
      break;

      case 6:
        harris3D->setMethod(pcl17::HarrisKeypoint3D<pcl17::PointXYZRGB,pcl17::PointXYZI>::CURVATURE);
      break;
      default:
        pcl17::console::print_error("unknown key point detection method %d\n expecting values between 1 and 6", keypoint_type);
        exit (1);
        break;
    }
    
  }
  
  boost::shared_ptr<pcl17::PCLSurfaceBase<pcl17::PointXYZRGBNormal> > surface_reconstruction;
  
  if (surface_type == 1)
  {
    pcl17::GreedyProjectionTriangulation<pcl17::PointXYZRGBNormal>* gp3 = new pcl17::GreedyProjectionTriangulation<pcl17::PointXYZRGBNormal>;

    // Set the maximum distance between connected points (maximum edge length)
    gp3->setSearchRadius (0.025);

    // Set typical values for the parameters
    gp3->setMu (2.5);
    gp3->setMaximumNearestNeighbors (100);
    gp3->setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3->setMinimumAngle(M_PI/18); // 10 degrees
    gp3->setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3->setNormalConsistency(false);
    surface_reconstruction.reset(gp3);
  }
  else if (surface_type == 2)
  {
    pcl17::MarchingCubes<pcl17::PointXYZRGBNormal>* mc = new pcl17::MarchingCubesHoppe<pcl17::PointXYZRGBNormal>;
    mc->setIsoLevel(0.001);
    mc->setGridResolution(50, 50, 50);
    surface_reconstruction.reset(mc);
  }
  else
  {
    pcl17::console::print_error("unknown surface reconstruction method %d\n expecting values between 1 and 2", surface_type);
    exit (1);
  }
  
  switch (descriptor_type)
  {
    case 1:
    {
      pcl17::Feature<pcl17::PointXYZRGB, pcl17::FPFHSignature33>::Ptr feature_extractor (new pcl17::FPFHEstimationOMP<pcl17::PointXYZRGB, pcl17::Normal, pcl17::FPFHSignature33>); 
      feature_extractor->setSearchMethod (pcl17::search::Search<pcl17::PointXYZRGB>::Ptr (new pcl17::search::KdTree<pcl17::PointXYZRGB>));
      feature_extractor->setRadiusSearch (0.05);
      ICCVTutorial<pcl17::FPFHSignature33> tutorial (keypoint_detector, feature_extractor, surface_reconstruction, source, target);
      tutorial.run ();
    }
    break;
    
    case 2:
    {
      pcl17::SHOTColorEstimationOMP<pcl17::PointXYZRGB, pcl17::Normal, pcl17::SHOT1344>* shot = new pcl17::SHOTColorEstimationOMP<pcl17::PointXYZRGB, pcl17::Normal, pcl17::SHOT1344>;
      shot->setRadiusSearch (0.04);
      pcl17::Feature<pcl17::PointXYZRGB, pcl17::SHOT1344>::Ptr feature_extractor (shot);
      ICCVTutorial<pcl17::SHOT1344> tutorial (keypoint_detector, feature_extractor, surface_reconstruction, source, target);
      tutorial.run ();
    }
    break;
    
    case 3:
    {
      pcl17::Feature<pcl17::PointXYZRGB, pcl17::PFHSignature125>::Ptr feature_extractor (new pcl17::PFHEstimation<pcl17::PointXYZRGB, pcl17::Normal, pcl17::PFHSignature125>);
      feature_extractor->setKSearch(50);
      ICCVTutorial<pcl17::PFHSignature125> tutorial (keypoint_detector, feature_extractor, surface_reconstruction, source, target);
      tutorial.run ();
    }
    break;
    
    case 4:
    {
      pcl17::Feature<pcl17::PointXYZRGB, pcl17::PFHRGBSignature250>::Ptr feature_extractor (new pcl17::PFHRGBEstimation<pcl17::PointXYZRGB, pcl17::Normal, pcl17::PFHRGBSignature250>);
      feature_extractor->setKSearch(50);
      ICCVTutorial<pcl17::PFHRGBSignature250> tutorial (keypoint_detector, feature_extractor, surface_reconstruction, source, target);
      tutorial.run ();
    }
    break;
    
    default:
      pcl17::console::print_error("unknown descriptor type %d\n expecting values between 1 and 4", descriptor_type);
      exit (1);
      break;
  }  
  
  return (0);
}
