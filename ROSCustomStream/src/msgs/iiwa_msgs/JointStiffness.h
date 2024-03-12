// Generated by gencpp from file iiwa_msgs/JointStiffness.msg
// DO NOT EDIT!


#ifndef IIWA_MSGS_MESSAGE_JOINTSTIFFNESS_H
#define IIWA_MSGS_MESSAGE_JOINTSTIFFNESS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <iiwa_msgs/JointQuantity.h>

namespace iiwa_msgs
{
template <class ContainerAllocator>
struct JointStiffness_
{
  typedef JointStiffness_<ContainerAllocator> Type;

  JointStiffness_()
    : header()
    , stiffness()  {
    }
  JointStiffness_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , stiffness(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::iiwa_msgs::JointQuantity_<ContainerAllocator>  _stiffness_type;
  _stiffness_type stiffness;





  typedef boost::shared_ptr< ::iiwa_msgs::JointStiffness_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::iiwa_msgs::JointStiffness_<ContainerAllocator> const> ConstPtr;

}; // struct JointStiffness_

typedef ::iiwa_msgs::JointStiffness_<std::allocator<void> > JointStiffness;

typedef boost::shared_ptr< ::iiwa_msgs::JointStiffness > JointStiffnessPtr;
typedef boost::shared_ptr< ::iiwa_msgs::JointStiffness const> JointStiffnessConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::iiwa_msgs::JointStiffness_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::iiwa_msgs::JointStiffness_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::iiwa_msgs::JointStiffness_<ContainerAllocator1> & lhs, const ::iiwa_msgs::JointStiffness_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.stiffness == rhs.stiffness;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::iiwa_msgs::JointStiffness_<ContainerAllocator1> & lhs, const ::iiwa_msgs::JointStiffness_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace iiwa_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::iiwa_msgs::JointStiffness_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::iiwa_msgs::JointStiffness_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::iiwa_msgs::JointStiffness_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::iiwa_msgs::JointStiffness_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::iiwa_msgs::JointStiffness_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::iiwa_msgs::JointStiffness_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::iiwa_msgs::JointStiffness_<ContainerAllocator> >
{
  static const char* value()
  {
    return "36eb539f80d8414d77a80b821ac39758";
  }

  static const char* value(const ::iiwa_msgs::JointStiffness_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x36eb539f80d8414dULL;
  static const uint64_t static_value2 = 0x77a80b821ac39758ULL;
};

template<class ContainerAllocator>
struct DataType< ::iiwa_msgs::JointStiffness_<ContainerAllocator> >
{
  static const char* value()
  {
    return "iiwa_msgs/JointStiffness";
  }

  static const char* value(const ::iiwa_msgs::JointStiffness_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::iiwa_msgs::JointStiffness_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"JointQuantity stiffness\n"
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
"\n"
"================================================================================\n"
"MSG: iiwa_msgs/JointQuantity\n"
"float32 a1\n"
"float32 a2\n"
"float32 a3\n"
"float32 a4\n"
"float32 a5\n"
"float32 a6\n"
"float32 a7\n"
;
  }

  static const char* value(const ::iiwa_msgs::JointStiffness_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::iiwa_msgs::JointStiffness_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.stiffness);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointStiffness_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::iiwa_msgs::JointStiffness_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::iiwa_msgs::JointStiffness_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "stiffness: ";
    s << std::endl;
    Printer< ::iiwa_msgs::JointQuantity_<ContainerAllocator> >::stream(s, indent + "  ", v.stiffness);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IIWA_MSGS_MESSAGE_JOINTSTIFFNESS_H
