#include <iostream>
#include <pcl17/io/pcd_io.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/features/feature.h>
#include <pcl17/visualization/cloud_viewer.h>
#include <pcl17/features/fpfh.h>
#include <pcl17/features/impl/fpfh.hpp>
#include <pcl17/recognition/implicit_shape_model.h>
#include <pcl17/recognition/impl/implicit_shape_model.hpp>

int
main (int argc, char** argv)
{
  if (argc == 0 || argc % 2 == 0)
    return (-1);

  unsigned int number_of_training_clouds = (argc - 3) / 2;

  pcl17::NormalEstimation<pcl17::PointXYZ, pcl17::Normal> normal_estimator;
  normal_estimator.setRadiusSearch (25.0);

  std::vector<pcl17::PointCloud<pcl17::PointXYZ>::Ptr> training_clouds;
  std::vector<pcl17::PointCloud<pcl17::Normal>::Ptr> training_normals;
  std::vector<unsigned int> training_classes;

  for (unsigned int i_cloud = 0; i_cloud < number_of_training_clouds - 1; i_cloud++)
  {
    pcl17::PointCloud<pcl17::PointXYZ>::Ptr tr_cloud(new pcl17::PointCloud<pcl17::PointXYZ> ());
    if ( pcl17::io::loadPCDFile <pcl17::PointXYZ> (argv[i_cloud * 2 + 1], *tr_cloud) == -1 )
      return (-1);

    pcl17::PointCloud<pcl17::Normal>::Ptr tr_normals = (new pcl17::PointCloud<pcl17::Normal>)->makeShared ();
    normal_estimator.setInputCloud (tr_cloud);
    normal_estimator.compute (*tr_normals);

    unsigned int tr_class = static_cast<unsigned int> (strtol (argv[i_cloud * 2 + 2], 0, 10));

    training_clouds.push_back (tr_cloud);
    training_normals.push_back (tr_normals);
    training_classes.push_back (tr_class);
  }

  pcl17::FPFHEstimation<pcl17::PointXYZ, pcl17::Normal, pcl17::Histogram<153> >::Ptr fpfh
    (new pcl17::FPFHEstimation<pcl17::PointXYZ, pcl17::Normal, pcl17::Histogram<153> >);
  fpfh->setRadiusSearch (30.0);
  pcl17::Feature< pcl17::PointXYZ, pcl17::Histogram<153> >::Ptr feature_estimator(fpfh);

  pcl17::ism::ImplicitShapeModelEstimation<153, pcl17::PointXYZ, pcl17::Normal> ism;
  ism.setFeatureEstimator(feature_estimator);
  ism.setTrainingClouds (training_clouds);
  ism.setTrainingNormals (training_normals);
  ism.setTrainingClasses (training_classes);
  ism.setSamplingSize (2.0f);

  pcl17::ism::ImplicitShapeModelEstimation<153, pcl17::PointXYZ, pcl17::Normal>::ISMModelPtr model = boost::shared_ptr<pcl17::features::ISMModel>
    (new pcl17::features::ISMModel);
  ism.trainISM (model);

  std::string file ("trained_ism_model.txt");
  model->saveModelToFile (file);

  model->loadModelFromfile (file);

  unsigned int testing_class = static_cast<unsigned int> (strtol (argv[argc - 1], 0, 10));
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr testing_cloud (new pcl17::PointCloud<pcl17::PointXYZ> ());
  if ( pcl17::io::loadPCDFile <pcl17::PointXYZ> (argv[argc - 2], *testing_cloud) == -1 )
    return (-1);

  pcl17::PointCloud<pcl17::Normal>::Ptr testing_normals = (new pcl17::PointCloud<pcl17::Normal>)->makeShared ();
  normal_estimator.setInputCloud (testing_cloud);
  normal_estimator.compute (*testing_normals);

  boost::shared_ptr<pcl17::features::ISMVoteList<pcl17::PointXYZ> > vote_list = ism.findObjects (
    model,
    testing_cloud,
    testing_normals,
    testing_class);

  double radius = model->sigmas_[testing_class] * 10.0;
  double sigma = model->sigmas_[testing_class];
  std::vector<pcl17::ISMPeak, Eigen::aligned_allocator<pcl17::ISMPeak> > strongest_peaks;
  vote_list->findStrongestPeaks (strongest_peaks, testing_class, radius, sigma);

  pcl17::PointCloud <pcl17::PointXYZRGB>::Ptr colored_cloud = (new pcl17::PointCloud<pcl17::PointXYZRGB>)->makeShared ();
  colored_cloud->height = 0;
  colored_cloud->width = 1;

  pcl17::PointXYZRGB point;
  point.r = 255;
  point.g = 255;
  point.b = 255;

  for (size_t i_point = 0; i_point < testing_cloud->points.size (); i_point++)
  {
    point.x = testing_cloud->points[i_point].x;
    point.y = testing_cloud->points[i_point].y;
    point.z = testing_cloud->points[i_point].z;
    colored_cloud->points.push_back (point);
  }
  colored_cloud->height += testing_cloud->points.size ();

  point.r = 255;
  point.g = 0;
  point.b = 0;
  for (size_t i_vote = 0; i_vote < strongest_peaks.size (); i_vote++)
  {
    point.x = strongest_peaks[i_vote].x;
    point.y = strongest_peaks[i_vote].y;
    point.z = strongest_peaks[i_vote].z;
    colored_cloud->points.push_back (point);
  }
  colored_cloud->height += strongest_peaks.size ();

  pcl17::visualization::CloudViewer viewer ("Result viewer");
  viewer.showCloud (colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  return (0);
}
