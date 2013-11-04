/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

// NOTE: This file has been created with 'pcl_src/geometry/include/pcl/geometry/mesh_indices.py'

#ifndef PCL17_GEOMETRY_MESH_INDICES_H
#define PCL17_GEOMETRY_MESH_INDICES_H

#include <iostream>

#include <pcl17/geometry/boost.h>

////////////////////////////////////////////////////////////////////////////////
// VertexIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl17
{
  namespace geometry
  {
    /** \brief Index used to access elements in the half-edge mesh. It is basically just a wrapper around an integer with a few added methods.
      * \author Martin Saelzle
      * \ingroup geometry
      */
    class VertexIndex
        : boost::totally_ordered <pcl17::geometry::VertexIndex // < > <= >= == !=
        , boost::unit_steppable  <pcl17::geometry::VertexIndex // ++ -- (pre and post)
        , boost::additive        <pcl17::geometry::VertexIndex // += + -= -
        > > >
    {
      public:

        typedef boost::totally_ordered <pcl17::geometry::VertexIndex,
                boost::unit_steppable  <pcl17::geometry::VertexIndex,
                boost::additive        <pcl17::geometry::VertexIndex> > > Base;
        typedef pcl17::geometry::VertexIndex                              Self;

        /** \brief Constructor. Initializes with an invalid index. */
        VertexIndex ()
          : index_ (-1)
        {
        }

        /** \brief Constructor.
          * \param[in] index The integer index.
          */
        explicit VertexIndex (const int index)
          : index_ (index)
        {
        }

        /** \brief Returns true if the index is valid. */
        inline bool
        isValid () const
        {
          return (index_ >= 0);
        }

        /** \brief Invalidate the index. */
        inline void
        invalidate ()
        {
          index_ = -1;
        }

        /** \brief Get the index. */
        inline int
        get () const
        {
          return (index_);
        }

        /** \brief Set the index. */
        inline void
        set (const int index)
        {
          index_ = index;
        }

        /** \brief Comparison operators (with boost::operators): < > <= >= */
        inline bool
        operator < (const Self& other) const
        {
          return (this->get () < other.get ());
        }

        /** \brief Comparison operators (with boost::operators): == != */
        inline bool
        operator == (const Self& other) const
        {
          return (this->get () == other.get ());
        }

        /** \brief Increment operators (with boost::operators): ++ (pre and post) */
        inline Self&
        operator ++ ()
        {
          ++index_;
          return (*this);
        }

        /** \brief Decrement operators (with boost::operators): \-\- (pre and post) */
        inline Self&
        operator -- ()
        {
          --index_;
          return (*this);
        }

        /** \brief Addition operators (with boost::operators): + += */
        inline Self&
        operator += (const Self& other)
        {
          index_ += other.get ();
          return (*this);
        }

        /** \brief Subtraction operators (with boost::operators): - -= */
        inline Self&
        operator -= (const Self& other)
        {
          index_ -= other.get ();
          return (*this);
        }

      private:

        /** \brief Stored index. */
        int index_;

        friend std::istream&
        operator >> (std::istream& is, pcl17::geometry::VertexIndex& index);
    };

    /** \brief ostream operator. */
    inline std::ostream&
    operator << (std::ostream& os, const pcl17::geometry::VertexIndex& index)
    {
      return (os << index.get ());
    }

    /** \brief istream operator. */
    inline std::istream&
    operator >> (std::istream& is, pcl17::geometry::VertexIndex& index)
    {
      return (is >> index.index_);
    }

  } // End namespace geometry
} // End namespace pcl17

////////////////////////////////////////////////////////////////////////////////
// HalfEdgeIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl17
{
  namespace geometry
  {
    /** \brief Index used to access elements in the half-edge mesh. It is basically just a wrapper around an integer with a few added methods.
      * \author Martin Saelzle
      * \ingroup geometry
      */
    class HalfEdgeIndex
        : boost::totally_ordered <pcl17::geometry::HalfEdgeIndex // < > <= >= == !=
        , boost::unit_steppable  <pcl17::geometry::HalfEdgeIndex // ++ -- (pre and post)
        , boost::additive        <pcl17::geometry::HalfEdgeIndex // += + -= -
        > > >
    {
      public:

        typedef boost::totally_ordered <pcl17::geometry::HalfEdgeIndex,
                boost::unit_steppable  <pcl17::geometry::HalfEdgeIndex,
                boost::additive        <pcl17::geometry::HalfEdgeIndex> > > Base;
        typedef pcl17::geometry::HalfEdgeIndex                              Self;

        /** \brief Constructor. Initializes with an invalid index. */
        HalfEdgeIndex ()
          : index_ (-1)
        {
        }

        /** \brief Constructor.
          * \param[in] index The integer index.
          */
        explicit HalfEdgeIndex (const int index)
          : index_ (index)
        {
        }

        /** \brief Returns true if the index is valid. */
        inline bool
        isValid () const
        {
          return (index_ >= 0);
        }

        /** \brief Invalidate the index. */
        inline void
        invalidate ()
        {
          index_ = -1;
        }

        /** \brief Get the index. */
        inline int
        get () const
        {
          return (index_);
        }

        /** \brief Set the index. */
        inline void
        set (const int index)
        {
          index_ = index;
        }

        /** \brief Comparison operators (with boost::operators): < > <= >= */
        inline bool
        operator < (const Self& other) const
        {
          return (this->get () < other.get ());
        }

        /** \brief Comparison operators (with boost::operators): == != */
        inline bool
        operator == (const Self& other) const
        {
          return (this->get () == other.get ());
        }

        /** \brief Increment operators (with boost::operators): ++ (pre and post) */
        inline Self&
        operator ++ ()
        {
          ++index_;
          return (*this);
        }

        /** \brief Decrement operators (with boost::operators): \-\- (pre and post) */
        inline Self&
        operator -- ()
        {
          --index_;
          return (*this);
        }

        /** \brief Addition operators (with boost::operators): + += */
        inline Self&
        operator += (const Self& other)
        {
          index_ += other.get ();
          return (*this);
        }

        /** \brief Subtraction operators (with boost::operators): - -= */
        inline Self&
        operator -= (const Self& other)
        {
          index_ -= other.get ();
          return (*this);
        }

      private:

        /** \brief Stored index. */
        int index_;

        friend std::istream&
        operator >> (std::istream& is, pcl17::geometry::HalfEdgeIndex& index);
    };

    /** \brief ostream operator. */
    inline std::ostream&
    operator << (std::ostream& os, const pcl17::geometry::HalfEdgeIndex& index)
    {
      return (os << index.get ());
    }

    /** \brief istream operator. */
    inline std::istream&
    operator >> (std::istream& is, pcl17::geometry::HalfEdgeIndex& index)
    {
      return (is >> index.index_);
    }

  } // End namespace geometry
} // End namespace pcl17

////////////////////////////////////////////////////////////////////////////////
// EdgeIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl17
{
  namespace geometry
  {
    /** \brief Index used to access elements in the half-edge mesh. It is basically just a wrapper around an integer with a few added methods.
      * \author Martin Saelzle
      * \ingroup geometry
      */
    class EdgeIndex
        : boost::totally_ordered <pcl17::geometry::EdgeIndex // < > <= >= == !=
        , boost::unit_steppable  <pcl17::geometry::EdgeIndex // ++ -- (pre and post)
        , boost::additive        <pcl17::geometry::EdgeIndex // += + -= -
        > > >
    {
      public:

        typedef boost::totally_ordered <pcl17::geometry::EdgeIndex,
                boost::unit_steppable  <pcl17::geometry::EdgeIndex,
                boost::additive        <pcl17::geometry::EdgeIndex> > > Base;
        typedef pcl17::geometry::EdgeIndex                              Self;

        /** \brief Constructor. Initializes with an invalid index. */
        EdgeIndex ()
          : index_ (-1)
        {
        }

        /** \brief Constructor.
          * \param[in] index The integer index.
          */
        explicit EdgeIndex (const int index)
          : index_ (index)
        {
        }

        /** \brief Returns true if the index is valid. */
        inline bool
        isValid () const
        {
          return (index_ >= 0);
        }

        /** \brief Invalidate the index. */
        inline void
        invalidate ()
        {
          index_ = -1;
        }

        /** \brief Get the index. */
        inline int
        get () const
        {
          return (index_);
        }

        /** \brief Set the index. */
        inline void
        set (const int index)
        {
          index_ = index;
        }

        /** \brief Comparison operators (with boost::operators): < > <= >= */
        inline bool
        operator < (const Self& other) const
        {
          return (this->get () < other.get ());
        }

        /** \brief Comparison operators (with boost::operators): == != */
        inline bool
        operator == (const Self& other) const
        {
          return (this->get () == other.get ());
        }

        /** \brief Increment operators (with boost::operators): ++ (pre and post) */
        inline Self&
        operator ++ ()
        {
          ++index_;
          return (*this);
        }

        /** \brief Decrement operators (with boost::operators): \-\- (pre and post) */
        inline Self&
        operator -- ()
        {
          --index_;
          return (*this);
        }

        /** \brief Addition operators (with boost::operators): + += */
        inline Self&
        operator += (const Self& other)
        {
          index_ += other.get ();
          return (*this);
        }

        /** \brief Subtraction operators (with boost::operators): - -= */
        inline Self&
        operator -= (const Self& other)
        {
          index_ -= other.get ();
          return (*this);
        }

      private:

        /** \brief Stored index. */
        int index_;

        friend std::istream&
        operator >> (std::istream& is, pcl17::geometry::EdgeIndex& index);
    };

    /** \brief ostream operator. */
    inline std::ostream&
    operator << (std::ostream& os, const pcl17::geometry::EdgeIndex& index)
    {
      return (os << index.get ());
    }

    /** \brief istream operator. */
    inline std::istream&
    operator >> (std::istream& is, pcl17::geometry::EdgeIndex& index)
    {
      return (is >> index.index_);
    }

  } // End namespace geometry
} // End namespace pcl17

////////////////////////////////////////////////////////////////////////////////
// FaceIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl17
{
  namespace geometry
  {
    /** \brief Index used to access elements in the half-edge mesh. It is basically just a wrapper around an integer with a few added methods.
      * \author Martin Saelzle
      * \ingroup geometry
      */
    class FaceIndex
        : boost::totally_ordered <pcl17::geometry::FaceIndex // < > <= >= == !=
        , boost::unit_steppable  <pcl17::geometry::FaceIndex // ++ -- (pre and post)
        , boost::additive        <pcl17::geometry::FaceIndex // += + -= -
        > > >
    {
      public:

        typedef boost::totally_ordered <pcl17::geometry::FaceIndex,
                boost::unit_steppable  <pcl17::geometry::FaceIndex,
                boost::additive        <pcl17::geometry::FaceIndex> > > Base;
        typedef pcl17::geometry::FaceIndex                              Self;

        /** \brief Constructor. Initializes with an invalid index. */
        FaceIndex ()
          : index_ (-1)
        {
        }

        /** \brief Constructor.
          * \param[in] index The integer index.
          */
        explicit FaceIndex (const int index)
          : index_ (index)
        {
        }

        /** \brief Returns true if the index is valid. */
        inline bool
        isValid () const
        {
          return (index_ >= 0);
        }

        /** \brief Invalidate the index. */
        inline void
        invalidate ()
        {
          index_ = -1;
        }

        /** \brief Get the index. */
        inline int
        get () const
        {
          return (index_);
        }

        /** \brief Set the index. */
        inline void
        set (const int index)
        {
          index_ = index;
        }

        /** \brief Comparison operators (with boost::operators): < > <= >= */
        inline bool
        operator < (const Self& other) const
        {
          return (this->get () < other.get ());
        }

        /** \brief Comparison operators (with boost::operators): == != */
        inline bool
        operator == (const Self& other) const
        {
          return (this->get () == other.get ());
        }

        /** \brief Increment operators (with boost::operators): ++ (pre and post) */
        inline Self&
        operator ++ ()
        {
          ++index_;
          return (*this);
        }

        /** \brief Decrement operators (with boost::operators): \-\- (pre and post) */
        inline Self&
        operator -- ()
        {
          --index_;
          return (*this);
        }

        /** \brief Addition operators (with boost::operators): + += */
        inline Self&
        operator += (const Self& other)
        {
          index_ += other.get ();
          return (*this);
        }

        /** \brief Subtraction operators (with boost::operators): - -= */
        inline Self&
        operator -= (const Self& other)
        {
          index_ -= other.get ();
          return (*this);
        }

      private:

        /** \brief Stored index. */
        int index_;

        friend std::istream&
        operator >> (std::istream& is, pcl17::geometry::FaceIndex& index);
    };

    /** \brief ostream operator. */
    inline std::ostream&
    operator << (std::ostream& os, const pcl17::geometry::FaceIndex& index)
    {
      return (os << index.get ());
    }

    /** \brief istream operator. */
    inline std::istream&
    operator >> (std::istream& is, pcl17::geometry::FaceIndex& index)
    {
      return (is >> index.index_);
    }

  } // End namespace geometry
} // End namespace pcl17

////////////////////////////////////////////////////////////////////////////////
// Conversions
////////////////////////////////////////////////////////////////////////////////

namespace pcl17
{
  namespace geometry
  {
    /** \brief Convert the given half-edge index to an edge index. */
    inline pcl17::geometry::EdgeIndex
    toEdgeIndex (const HalfEdgeIndex& index)
    {
      return (index.isValid () ? EdgeIndex (index.get () / 2) : EdgeIndex ());
    }

    /** \brief Convert the given edge index to a half-edge index.
      * \param[in] get_first The first half-edge of the edge is returned if this variable is true; elsewise the second.
      */
    inline pcl17::geometry::HalfEdgeIndex
    toHalfEdgeIndex (const EdgeIndex& index, const bool get_first=true)
    {
      return (index.isValid () ? HalfEdgeIndex (index.get () * 2 + static_cast <int> (!get_first)) : HalfEdgeIndex ());
    }
  } // End namespace geometry
} // End namespace pcl17

#endif // PCL17_GEOMETRY_MESH_INDICES_H
