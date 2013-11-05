/* Auto-generated by genmsg_cpp for file /home/micsu/fuerte_workspace/fsr2013/fuerte-unstable-devel/pcl17/msg/ModelCoefficients.msg */
#ifndef PCL17_MESSAGE_MODELCOEFFICIENTS_H
#define PCL17_MESSAGE_MODELCOEFFICIENTS_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace pcl17
{
template <class ContainerAllocator>
struct ModelCoefficients_ {
  typedef ModelCoefficients_<ContainerAllocator> Type;

  ModelCoefficients_()
  : header()
  , values()
  {
  }

  ModelCoefficients_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , values(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _values_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  values;


  typedef boost::shared_ptr< ::pcl17::ModelCoefficients_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pcl17::ModelCoefficients_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ModelCoefficients
typedef  ::pcl17::ModelCoefficients_<std::allocator<void> > ModelCoefficients;

typedef boost::shared_ptr< ::pcl17::ModelCoefficients> ModelCoefficientsPtr;
typedef boost::shared_ptr< ::pcl17::ModelCoefficients const> ModelCoefficientsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pcl17::ModelCoefficients_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pcl17::ModelCoefficients_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pcl17

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pcl17::ModelCoefficients_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pcl17::ModelCoefficients_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pcl17::ModelCoefficients_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ca27dea75e72cb894cd36f9e5005e93e";
  }

  static const char* value(const  ::pcl17::ModelCoefficients_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xca27dea75e72cb89ULL;
  static const uint64_t static_value2 = 0x4cd36f9e5005e93eULL;
};

template<class ContainerAllocator>
struct DataType< ::pcl17::ModelCoefficients_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pcl17/ModelCoefficients";
  }

  static const char* value(const  ::pcl17::ModelCoefficients_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pcl17::ModelCoefficients_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float32[] values\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::pcl17::ModelCoefficients_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::pcl17::ModelCoefficients_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::pcl17::ModelCoefficients_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pcl17::ModelCoefficients_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.values);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ModelCoefficients_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pcl17::ModelCoefficients_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pcl17::ModelCoefficients_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "values[]" << std::endl;
    for (size_t i = 0; i < v.values.size(); ++i)
    {
      s << indent << "  values[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.values[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // PCL17_MESSAGE_MODELCOEFFICIENTS_H
