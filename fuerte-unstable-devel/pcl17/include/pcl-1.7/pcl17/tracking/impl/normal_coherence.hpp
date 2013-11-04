#ifndef PCL17_TRACKING_IMPL_NORMAL_COHERENCE_H_
#define PCL17_TRACKING_IMPL_NORMAL_COHERENCE_H_

#include <pcl17/common/common.h>
#include <pcl17/console/print.h>
#include <pcl17/tracking/normal_coherence.h>

template <typename PointInT> double 
pcl17::tracking::NormalCoherence<PointInT>::computeCoherence (PointInT &source, PointInT &target)
{
    Eigen::Vector4f n = source.getNormalVector4fMap ();
    Eigen::Vector4f n_dash = target.getNormalVector4fMap ();
    if ( n.norm () <= 1e-5 || n_dash.norm () <= 1e-5 )
    {
        PCL17_ERROR("norm might be ZERO!\n");
        std::cout << "source: " << source << std::endl;
        std::cout << "target: " << target << std::endl;
        exit (1);
        return 0.0;
    }
    else
    {
        n.normalize ();
        n_dash.normalize ();
        double theta = pcl17::getAngle3D (n, n_dash);
        if (!pcl_isnan (theta))
            return 1.0 / (1.0 + weight_ * theta * theta);
        else
            return 0.0;
    }
}


#define PCL17_INSTANTIATE_NormalCoherence(T) template class PCL17_EXPORTS pcl17::tracking::NormalCoherence<T>;

#endif
