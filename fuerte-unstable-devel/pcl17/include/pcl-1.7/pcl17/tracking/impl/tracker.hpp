#ifndef PCL17_TRACKING_IMPL_TRACKER_H_
#define PCL17_TRACKING_IMPL_TRACKER_H_

#include <pcl17/common/eigen.h>
#include <ctime>
#include <pcl17/tracking/boost.h>
#include <pcl17/tracking/tracker.h>

template <typename PointInT, typename StateT> bool
pcl17::tracking::Tracker<PointInT, StateT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
  {
    PCL17_ERROR ("[pcl17::%s::initCompute] PCLBase::Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // If the dataset is empty, just return
  if (input_->points.empty ())
  {
    PCL17_ERROR ("[pcl17::%s::compute] input_ is empty!\n", getClassName ().c_str ());
    // Cleanup
    deinitCompute ();
    return (false);
  }

  return (true);
}

template <typename PointInT, typename StateT> void
pcl17::tracking::Tracker<PointInT, StateT>::compute ()
{
  if (!initCompute ())
    return;
  
  computeTracking ();
  deinitCompute ();
}

#endif
