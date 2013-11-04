#include <pcl17/common/distances.h>
#include <pcl17/console/parse.h>
#include <pcl17/console/time.h>
#include <pcl17/point_types.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/filters/passthrough.h>
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/visualization/pcl_visualizer.h>
#include <pcl17/filters/conditional_removal.h>
#include <pcl17/segmentation/extract_clusters.h>

#define MIN_NR_INLIERS_LINE 40

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
class ConditionThresholdHSV : public pcl17::ConditionBase<PointT>
{
  public:
    typedef typename boost::shared_ptr<ConditionThresholdHSV<PointT> > Ptr;
    
    ConditionThresholdHSV (float min_h, float max_h, float min_s, float max_s, float min_v, float max_v) :
      min_h_(min_h), max_h_(max_h), min_s_(min_s), max_s_(max_s), min_v_(min_v), max_v_(max_v)
    {
      // Make min_h_ and max_h_ fall within [0, 360)
      assert (!pcl_isnan(min_h) && !pcl_isnan(max_h));
      while (min_h_ < 0) min_h_ += 360;
      while (min_h_ >= 360) min_h_ -= 360;
      while (max_h_ < 0) max_h_ += 360;
      while (max_h_ >= 360) max_h_ -= 360;
    }
    
    // Evaluate whether the color of the given point falls within the specified thresholds
    virtual bool evaluate(const PointT & p) const
    {
      float h, s, v;
      rgb2hsv (p.r, p.g, p.b, h, s, v);
      return (!pcl_isnan(h) && !pcl_isnan(s) && !pcl_isnan(v) && 
              ((min_h_ < max_h_) ? ((min_h_ <= h) && (h <= max_h_)) : ((min_h_ <= h) || (h <= max_h_))) &&
              (min_s_ <= s) && (s <= max_s_) &&
              (min_v_ <= v) && (v <= max_v_));
    }
    
    void rgb2hsv (uint8_t r, uint8_t g, uint8_t b, float & h, float & s, float & v) const
    {
      float maxval = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
      float minval = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
      float minmaxdiff = maxval - minval;
      
      if (maxval == minval)
      {
        h = 0;
        s = 0;
        v = maxval;
        return;
      }   
      else if (maxval == r)
      {
        h = 60.0*((g - b)/minmaxdiff);
        if (h < 0) h += 360.0;
      }
      else if (maxval == g)
      {
        h = 60.0*((b - r)/minmaxdiff + 2.0);
      }
      else // (maxval == b)
      {
        h = 60.0*((r - g)/minmaxdiff + 4.0);
      }
      s = 100.0 * minmaxdiff / maxval;
      v = maxval;
    }

  protected:
    float min_h_, max_h_, min_s_, max_s_, min_v_, max_v_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
filterRed (const pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr &input, pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr &output)
{
  pcl17::ConditionalRemoval<pcl17::PointXYZRGB> removal_filter;
  removal_filter.setKeepOrganized (false);
  ConditionThresholdHSV<pcl17::PointXYZRGB>::Ptr condition (new ConditionThresholdHSV<pcl17::PointXYZRGB> (-20,20, 75,100, 25,255));
  removal_filter.setCondition (condition);

  removal_filter.setInputCloud (input);
  removal_filter.filter (*output);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
filterGreen (const pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr &input, pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr &output)
{
  pcl17::ConditionalRemoval<pcl17::PointXYZRGB> removal_filter;
  removal_filter.setKeepOrganized (false);
  ConditionThresholdHSV<pcl17::PointXYZRGB>::Ptr condition (new ConditionThresholdHSV<pcl17::PointXYZRGB> (90,150, 15,100, 25,255));
  removal_filter.setCondition (condition);

  removal_filter.setInputCloud (input);
  removal_filter.filter (*output);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
downsample (const pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr &input, 
            pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr &output)
{
  pcl17::VoxelGrid<pcl17::PointXYZRGB> pass;
  pass.setInputCloud (input);
  pass.setLeafSize (0.005, 0.005, 0.005);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 2.0);
  pass.filter (*output);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
extractLargestCluster (const pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr &input, 
                       const pcl17::PointIndices::Ptr &inliers_all,
                       pcl17::PointIndices &inliers)
{
  pcl17::EuclideanClusterExtraction<pcl17::PointXYZRGB> ece;
  ece.setInputCloud (input);
  ece.setIndices (inliers_all);
  ece.setClusterTolerance (0.3);   // 30cm cluster separation
  std::vector<pcl17::PointIndices> clusters;
  ece.extract (clusters);
  inliers = clusters[0];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
compute (const pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr &input, 
         pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr &output,
         pcl17::ModelCoefficients &coefficients,
         pcl17::PointIndices &inliers)
{
  // Filter
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr output_down (new pcl17::PointCloud<pcl17::PointXYZRGB>);
  downsample (input, output_down);

  if (output_down->points.empty ())
  {
    inliers.indices.clear ();
    coefficients.values.clear ();
    return;
  }
  filterGreen (output_down, output);

  if (output->points.empty ())
  {
    inliers.indices.clear ();
    coefficients.values.clear ();
    return;
  }
  pcl17::SACSegmentation<pcl17::PointXYZRGB> seg;
  seg.setInputCloud (output);
  seg.setOptimizeCoefficients (false);
  seg.setProbability (0.99);
  seg.setMaxIterations (10000);
  seg.setModelType (pcl17::SACMODEL_STICK);
  seg.setMethodType (pcl17::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);
  //seg.setRadiusLimits (0.02, 0.08);
  pcl17::PointIndices::Ptr inliers_all (new pcl17::PointIndices);
  seg.segment (*inliers_all, coefficients);
  if (inliers_all->indices.size () < MIN_NR_INLIERS_LINE)
  {
    inliers.indices.clear ();
    coefficients.values.clear ();
    return;
  }

  extractLargestCluster (output, inliers_all, inliers);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
main (int argc, char** argv)
{
  srand (time (0));

  pcl17::visualization::PCLVisualizer p (argc, argv, "Line segmentation");

  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZRGB>);
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud_d (new pcl17::PointCloud<pcl17::PointXYZRGB>);
  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud_f (new pcl17::PointCloud<pcl17::PointXYZRGB>);
  pcl17::ModelCoefficients coefficients;
  pcl17::PointIndices inliers;

  std::vector<int> p_file_indices = pcl17::console::parse_file_extension_argument (argc, argv, ".pcd");

  for (size_t i = 0; i < p_file_indices.size (); ++i)
  {
    pcl17::io::loadPCDFile (argv[p_file_indices[i]], *cloud);
    
    // Compute
    pcl17::console::TicToc tt;
    tt.tic ();
    compute (cloud, cloud_f, coefficients, inliers);
    tt.toc_print ();

    if (inliers.indices.empty ())
    {
      p.removeShape ("line");
      continue;
    }

    // Display
    PCL17_INFO ("Found %zu inliers.\n", inliers.indices.size ());

    pcl17::PointCloud<pcl17::PointXYZ>::Ptr line (new pcl17::PointCloud<pcl17::PointXYZ>);
    pcl17::copyPointCloud (*cloud_f, inliers, *line);

    if (!p.updatePointCloud (cloud, "all"))
    {
      p.addPointCloud (cloud, "all");
      p.resetCameraViewpoint ("all");
    }

    if (!p.updatePointCloud (cloud_f, "filter"))
      p.addPointCloud (cloud_f, "filter");
    p.setPointCloudRenderingProperties (pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 10.0, "filter");
    p.setPointCloudRenderingProperties (pcl17::visualization::PCL17_VISUALIZER_OPACITY, 0.2, "filter");

    if (!p.updatePointCloud (line, "line inliers")) 
      p.addPointCloud (line, "line inliers");

    pcl17::PointXYZRGB pmin, pmax;
    if (pcl17::getMaxSegment (*cloud_f, inliers.indices, pmin, pmax) != std::numeric_limits<double>::min ())
      p.addLine<pcl17::PointXYZRGB> (pmin, pmax);
    else
    {
      pmin.x = coefficients.values[0]; pmin.y = coefficients.values[1]; pmin.z = coefficients.values[2];
      pmax.x = coefficients.values[3]; pmax.y = coefficients.values[4]; pmax.z = coefficients.values[5];
      PCL17_ERROR ("Couldn't compute the maximum segment!\n");
      p.addLine<pcl17::PointXYZRGB> (pmin, pmax);
      //p.addLine (coefficients);
    }
    p.setShapeRenderingProperties (pcl17::visualization::PCL17_VISUALIZER_LINE_WIDTH, 50.0, "line");
    p.setShapeRenderingProperties (pcl17::visualization::PCL17_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "line");

    if (p_file_indices.size () == 1)
      p.spin ();
    p.spinOnce ();
    p.removeShape ("line");
  }

  if (p_file_indices.size () != 1)
    p.spin ();

  return (0);
}
