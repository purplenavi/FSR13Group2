#ifndef PCL17_TRACKING_NORMAL_COHERENCE_H_
#define PCL17_TRACKING_NORMAL_COHERENCE_H_

#include <pcl17/tracking/coherence.h>
namespace pcl17
{
  namespace tracking
  {
    /** \brief @b NormalCoherence computes coherence between two points from the angle
        between their normals. the coherence is calculated by 1 / (1 + weight * theta^2 ).
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT>
    class NormalCoherence: public PointCoherence<PointInT>
    {
    public:

      /** \brief initialize the weight to 1.0. */
      NormalCoherence ()
      : PointCoherence<PointInT> ()
      , weight_ (1.0)
        {}

      /** \brief set the weight of coherence
        * \param weight the weight of coherence
        */
      inline void setWeight (double weight) { weight_ = weight; }

      /** \brief get the weight of coherence */
      inline double getWeight () { return weight_; }

    protected:

      /** \brief return the normal coherence between the two points.
        * \param source instance of source point.
        * \param target instance of target point.
        */
      double computeCoherence (PointInT &source, PointInT &target);

      /** \brief the weight of coherence */
      double weight_;
      
    };
  }
}

// #include <pcl17/tracking/impl/normal_coherence.hpp>
#ifdef PCL17_NO_PRECOMPILE
#include <pcl17/tracking/impl/normal_coherence.hpp>
#endif

#endif
