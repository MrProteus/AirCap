// Generated by gencpp from file read_omni_dataset/BallData.msg
// DO NOT EDIT!


#ifndef READ_OMNI_DATASET_MESSAGE_BALLDATA_H
#define READ_OMNI_DATASET_MESSAGE_BALLDATA_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace read_omni_dataset
{
template <class ContainerAllocator>
struct BallData_
{
  typedef BallData_<ContainerAllocator> Type;

  BallData_()
    : header()
    , found(false)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , BlobSize(0.0)
    , blobXCenterPixel(0.0)
    , blobYCenterPixel(0.0)
    , mismatchFactor(0.0)  {
    }
  BallData_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , found(false)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , BlobSize(0.0)
    , blobXCenterPixel(0.0)
    , blobYCenterPixel(0.0)
    , mismatchFactor(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _found_type;
  _found_type found;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef float _BlobSize_type;
  _BlobSize_type BlobSize;

   typedef float _blobXCenterPixel_type;
  _blobXCenterPixel_type blobXCenterPixel;

   typedef float _blobYCenterPixel_type;
  _blobYCenterPixel_type blobYCenterPixel;

   typedef float _mismatchFactor_type;
  _mismatchFactor_type mismatchFactor;







  typedef boost::shared_ptr< ::read_omni_dataset::BallData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::read_omni_dataset::BallData_<ContainerAllocator> const> ConstPtr;

}; // struct BallData_

typedef ::read_omni_dataset::BallData_<std::allocator<void> > BallData;

typedef boost::shared_ptr< ::read_omni_dataset::BallData > BallDataPtr;
typedef boost::shared_ptr< ::read_omni_dataset::BallData const> BallDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::read_omni_dataset::BallData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::read_omni_dataset::BallData_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::read_omni_dataset::BallData_<ContainerAllocator1> & lhs, const ::read_omni_dataset::BallData_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.found == rhs.found &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.BlobSize == rhs.BlobSize &&
    lhs.blobXCenterPixel == rhs.blobXCenterPixel &&
    lhs.blobYCenterPixel == rhs.blobYCenterPixel &&
    lhs.mismatchFactor == rhs.mismatchFactor;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::read_omni_dataset::BallData_<ContainerAllocator1> & lhs, const ::read_omni_dataset::BallData_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace read_omni_dataset

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::read_omni_dataset::BallData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::read_omni_dataset::BallData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::read_omni_dataset::BallData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::read_omni_dataset::BallData_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::read_omni_dataset::BallData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::read_omni_dataset::BallData_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::read_omni_dataset::BallData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "161090ceadc5449a9fe867d5bfd7bba3";
  }

  static const char* value(const ::read_omni_dataset::BallData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x161090ceadc5449aULL;
  static const uint64_t static_value2 = 0x9fe867d5bfd7bba3ULL;
};

template<class ContainerAllocator>
struct DataType< ::read_omni_dataset::BallData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "read_omni_dataset/BallData";
  }

  static const char* value(const ::read_omni_dataset::BallData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::read_omni_dataset::BallData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"bool found\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 BlobSize\n"
"float32 blobXCenterPixel\n"
"float32 blobYCenterPixel\n"
"float32 mismatchFactor\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::read_omni_dataset::BallData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::read_omni_dataset::BallData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.found);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.BlobSize);
      stream.next(m.blobXCenterPixel);
      stream.next(m.blobYCenterPixel);
      stream.next(m.mismatchFactor);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BallData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::read_omni_dataset::BallData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::read_omni_dataset::BallData_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "found: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.found);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "BlobSize: ";
    Printer<float>::stream(s, indent + "  ", v.BlobSize);
    s << indent << "blobXCenterPixel: ";
    Printer<float>::stream(s, indent + "  ", v.blobXCenterPixel);
    s << indent << "blobYCenterPixel: ";
    Printer<float>::stream(s, indent + "  ", v.blobYCenterPixel);
    s << indent << "mismatchFactor: ";
    Printer<float>::stream(s, indent + "  ", v.mismatchFactor);
  }
};

} // namespace message_operations
} // namespace ros

#endif // READ_OMNI_DATASET_MESSAGE_BALLDATA_H
