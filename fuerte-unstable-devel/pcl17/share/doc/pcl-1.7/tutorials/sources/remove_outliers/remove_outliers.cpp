#include <iostream>
#include <pcl17/point_types.h>
#include <pcl17/filters/radius_outlier_removal.h>
#include <pcl17/filters/conditional_removal.h>

int
 main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>);
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_filtered (new pcl17::PointCloud<pcl17::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  if (strcmp(argv[1], "-r") == 0){
    pcl17::RadiusOutlierRemoval<pcl17::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (2);
    // apply filter
    outrem.filter (*cloud_filtered);
  }
  else if (strcmp(argv[1], "-c") == 0){
    // build the condition
    pcl17::ConditionAnd<pcl17::PointXYZ>::Ptr range_cond (new
      pcl17::ConditionAnd<pcl17::PointXYZ> ());
    range_cond->addComparison (pcl17::FieldComparison<pcl17::PointXYZ>::ConstPtr (new
      pcl17::FieldComparison<pcl17::PointXYZ> ("z", pcl17::ComparisonOps::GT, 0.0)));
    range_cond->addComparison (pcl17::FieldComparison<pcl17::PointXYZ>::ConstPtr (new
      pcl17::FieldComparison<pcl17::PointXYZ> ("z", pcl17::ComparisonOps::LT, 0.8)));
    // build the filter
    pcl17::ConditionalRemoval<pcl17::PointXYZ> condrem (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (*cloud_filtered);
  }
  else{
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;
  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " "
                        << cloud_filtered->points[i].y << " "
                        << cloud_filtered->points[i].z << std::endl;
  return (0);
}