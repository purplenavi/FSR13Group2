#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <pcl17/point_types.h>
#include <pcl17/point_cloud.h>

/*  Define some custom types to make the rest of our code easier to read */

// Define "PointCloud" to be a pcl17::PointCloud of pcl17::PointXYZRGB points
typedef pcl17::PointXYZRGB PointT;
typedef pcl17::PointCloud<PointT> PointCloud;
typedef pcl17::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl17::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

// Define "SurfaceNormals" to be a pcl17::PointCloud of pcl17::Normal points
typedef pcl17::Normal NormalT;
typedef pcl17::PointCloud<NormalT> SurfaceNormals;
typedef pcl17::PointCloud<NormalT>::Ptr SurfaceNormalsPtr;
typedef pcl17::PointCloud<NormalT>::ConstPtr SurfaceNormalsConstPtr;

// Define "SurfaceElements" to be a pcl17::PointCloud of pcl17::PointNormal points
typedef pcl17::PointNormal SurfelT;
typedef pcl17::PointCloud<SurfelT> SurfaceElements;
typedef pcl17::PointCloud<SurfelT>::Ptr SurfaceElementsPtr;
typedef pcl17::PointCloud<SurfelT>::ConstPtr SurfaceElementsConstPtr;


// Define "LocalDescriptors" to be a pcl17::PointCloud of pcl17::FPFHSignature33 points
typedef pcl17::FPFHSignature33 LocalDescriptorT;
typedef pcl17::PointCloud<LocalDescriptorT> LocalDescriptors;
typedef pcl17::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;
typedef pcl17::PointCloud<LocalDescriptorT>::ConstPtr LocalDescriptorsConstPtr;

// Define "GlobalDescriptors" to be a pcl17::PointCloud of pcl17::VFHSignature308 points
typedef pcl17::VFHSignature308 GlobalDescriptorT;
typedef pcl17::PointCloud<GlobalDescriptorT> GlobalDescriptors;
typedef pcl17::PointCloud<GlobalDescriptorT>::Ptr GlobalDescriptorsPtr;
typedef pcl17::PointCloud<GlobalDescriptorT>::ConstPtr GlobalDescriptorsConstPtr;

#endif
