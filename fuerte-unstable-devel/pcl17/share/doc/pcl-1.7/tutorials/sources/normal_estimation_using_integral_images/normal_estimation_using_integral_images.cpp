#include <pcl17/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl17/io/io.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/features/integral_image_normal.h>
    
int 
main ()
{
    // load point cloud
    pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>);
    pcl17::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);
    
    // estimate normals
    pcl17::PointCloud<pcl17::Normal>::Ptr normals (new pcl17::PointCloud<pcl17::Normal>);

    pcl17::IntegralImageNormalEstimation<pcl17::PointXYZ, pcl17::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // visualize normals
    pcl17::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl17::PointXYZ,pcl17::Normal>(cloud, normals);
    
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }
    return 0;
}
