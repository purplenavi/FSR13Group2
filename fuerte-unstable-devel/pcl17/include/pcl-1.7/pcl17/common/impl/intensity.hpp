/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL17_COMMON_INTENSITY_FIELD_ACCESSOR_IMPL_HPP
#define PCL17_COMMON_INTENSITY_FIELD_ACCESSOR_IMPL_HPP

#include <pcl17/point_types.h>
namespace pcl17
{
  namespace common
  {
    template<>
    struct IntensityFieldAccessor<pcl17::PointNormal>
    {
      inline float
      operator () (const pcl17::PointNormal &p) const
      {
        return (p.curvature);
      }

      inline void
      get (const pcl17::PointNormal &p, float &intensity) const
      {
        intensity = p.curvature;
      }

      inline void
      set (pcl17::PointNormal &p, float intensity) const
      {
        p.curvature = intensity;
      }

      inline void
      demean (pcl17::PointNormal& p, float value) const
      {
        p.curvature -= value;
      }

      inline void
      add (pcl17::PointNormal& p, float value) const
      {
        p.curvature += value;
      }
    };
    
    template<>
    struct IntensityFieldAccessor<pcl17::PointXYZ>
    {
      inline float
      operator () (const pcl17::PointXYZ &p) const
      {
        return (p.z);
      }

      inline void
      get (const pcl17::PointXYZ &p, float &intensity) const
      {
        intensity = p.z;
      }

      inline void
      set (pcl17::PointXYZ &p, float intensity) const
      {
        p.z = intensity;
      }

      inline void
      demean (pcl17::PointXYZ& p, float value) const
      {
        p.z -= value;
      }

      inline void
      add (pcl17::PointXYZ& p, float value) const
      {
        p.z += value;
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl17::PointXYZRGB>
    {
      inline float
      operator () (const pcl17::PointXYZRGB &p) const
      {
        return (static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f);
      }

      inline void
      get (const pcl17::PointXYZRGB &p, float& intensity) const
      {
        intensity = static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f;
      }
      
      inline void
      set (pcl17::PointXYZRGB &p, float intensity) const
      {
        p.r = static_cast<uint8_t> (intensity * 3.34448160535f); // 1000 / 299
        p.g = static_cast<uint8_t> (intensity * 1.70357751278f); // 1000 / 587
        p.b = static_cast<uint8_t> (intensity * 8.77192982456f); // 1000 / 114
      }
      
      inline void
      demean (pcl17::PointXYZRGB& p, float value) const
      {
        float intensity = this->operator () (p);
        intensity -= value;
        set (p, intensity);
      }
      
      inline void
      add (pcl17::PointXYZRGB& p, float value) const
      {
        float intensity = this->operator () (p);
        intensity += value;
        set (p, intensity);
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl17::PointXYZRGBA>
    {
      inline float
      operator () (const pcl17::PointXYZRGBA &p) const
      {
        return (static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f);
      }
      
      inline void
      get (const pcl17::PointXYZRGBA &p, float& intensity) const
      {
        intensity = static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f;
      }

      inline void
      set (pcl17::PointXYZRGBA &p, float intensity) const
      {
        p.r = static_cast<uint8_t> (intensity * 3.34448160535f); // 1000 / 299
        p.g = static_cast<uint8_t> (intensity * 1.70357751278f); // 1000 / 587
        p.b = static_cast<uint8_t> (intensity * 8.77192982456f); // 1000 / 114
      }
      
      inline void
      demean (pcl17::PointXYZRGBA& p, float value) const
      {
        float intensity = this->operator () (p);
        intensity -= value;
        set (p, intensity);
      }
      
      inline void
      add (pcl17::PointXYZRGBA& p, float value) const
      {
        float intensity = this->operator () (p);
        intensity += value;
        set (p, intensity);
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl17::PointXYZRGBNormal>
    {
      inline float
      operator () (const pcl17::PointXYZRGBNormal &p) const
      {
        return (static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f);
      }
      
      inline void
      get (const pcl17::PointXYZRGBNormal &p, float& intensity) const
      {
        intensity = static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f;
      }

      inline void
      set (pcl17::PointXYZRGBNormal &p, float intensity) const
      {
        p.r = static_cast<uint8_t> (intensity * 3.34448160535f); // 1000 / 299
        p.g = static_cast<uint8_t> (intensity * 1.70357751278f); // 1000 / 587
        p.b = static_cast<uint8_t> (intensity * 8.77192982456f); // 1000 / 114
      }
      
      inline void
      demean (pcl17::PointXYZRGBNormal &p, float value) const
      {
        float intensity = this->operator () (p);
        intensity -= value;
        set (p, intensity);
      }
      
      inline void
      add (pcl17::PointXYZRGBNormal &p, float value) const
      {
        float intensity = this->operator () (p);
        intensity += value;
        set (p, intensity);
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl17::PointXYZRGBL>
    {
      inline float
      operator () (const pcl17::PointXYZRGBL &p) const
      {
        return (static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f);
      }

      inline void
      get (const pcl17::PointXYZRGBL &p, float& intensity) const
      {
        intensity = static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f;
      }
      
      inline void
      set (pcl17::PointXYZRGBL &p, float intensity) const
      {
        p.r = static_cast<uint8_t> (intensity * 3.34448160535f); // 1000 / 299
        p.g = static_cast<uint8_t> (intensity * 1.70357751278f); // 1000 / 587
        p.b = static_cast<uint8_t> (intensity * 8.77192982456f); // 1000 / 114
      }
      
      inline void
      demean (pcl17::PointXYZRGBL& p, float value) const
      {
        float intensity = this->operator () (p);
        intensity -= value;
        set (p, intensity);
      }
      
      inline void
      add (pcl17::PointXYZRGBL& p, float value) const
      {
        float intensity = this->operator () (p);
        intensity += value;
        set (p, intensity);
      }
    };
  }
}

#endif
