#include <iostream>
#include <pcl17/io/pcd_io.h>
#include <pcl17/point_types.h>

int
main (int argc, char** argv)
{
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>);

  if (pcl17::io::loadPCDFile<pcl17::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL17_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;

  return (0);
}
