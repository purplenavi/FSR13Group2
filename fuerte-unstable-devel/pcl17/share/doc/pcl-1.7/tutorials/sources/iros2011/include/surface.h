#ifndef SURFACE_H_
#define SURFACE_H_

#include <pcl17/kdtree/kdtree_flann.h>
#include <pcl17/surface/mls.h>
#include <pcl17/surface/convex_hull.h>
#include <pcl17/surface/concave_hull.h>
#include <pcl17/surface/gp3.h>
#include <pcl17/surface/marching_cubes_hoppe.h>

#include "typedefs.h"


class Mesh
{
public:
  Mesh () : points (new PointCloud) {}
  PointCloudPtr points;
  std::vector<pcl17::Vertices> faces;
};

typedef boost::shared_ptr<Mesh> MeshPtr;

PointCloudPtr
smoothPointCloud (const PointCloudPtr & input, float radius, int polynomial_order)
{
  PointCloudPtr output (new PointCloud);
  return (output);
}

SurfaceElementsPtr
computeSurfaceElements (const PointCloudPtr & input, float radius, int polynomial_order)
{
  SurfaceElementsPtr surfels (new SurfaceElements);
  return (surfels);
}

MeshPtr
computeConvexHull (const PointCloudPtr & input)
{
  MeshPtr output (new Mesh);
  return (output);
}


MeshPtr
computeConcaveHull (const PointCloudPtr & input, float alpha)
{
  MeshPtr output (new Mesh);
  return (output);
}

pcl17::PolygonMesh::Ptr
greedyTriangulation (const SurfaceElementsPtr & surfels, float radius, float mu, int max_nearest_neighbors, 
                     float max_surface_angle, float min_angle, float max_angle)

{
  pcl17::PolygonMesh::Ptr output (new pcl17::PolygonMesh);
  return (output);
}


pcl17::PolygonMesh::Ptr
marchingCubesTriangulation (const SurfaceElementsPtr & surfels, float leaf_size, float iso_level)
{
  pcl17::PolygonMesh::Ptr output (new pcl17::PolygonMesh);
  return (output);
}

#endif
