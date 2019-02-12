// Generated by gencpp from file rtabmap_ros/ListLabelsResponse.msg
// DO NOT EDIT!


#ifndef RTABMAP_ROS_MESSAGE_LISTLABELSRESPONSE_H
#define RTABMAP_ROS_MESSAGE_LISTLABELSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rtabmap_ros
{
template <class ContainerAllocator>
struct ListLabelsResponse_
{
  typedef ListLabelsResponse_<ContainerAllocator> Type;

  ListLabelsResponse_()
    : labels()  {
    }
  ListLabelsResponse_(const ContainerAllocator& _alloc)
    : labels(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _labels_type;
  _labels_type labels;





  typedef boost::shared_ptr< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ListLabelsResponse_

typedef ::rtabmap_ros::ListLabelsResponse_<std::allocator<void> > ListLabelsResponse;

typedef boost::shared_ptr< ::rtabmap_ros::ListLabelsResponse > ListLabelsResponsePtr;
typedef boost::shared_ptr< ::rtabmap_ros::ListLabelsResponse const> ListLabelsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rtabmap_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'rtabmap_ros': ['/home/mars/Software/ros/src/src/rtabmap_ros/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "17b562487ca772bdfa2c078ef00d870f";
  }

  static const char* value(const ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x17b562487ca772bdULL;
  static const uint64_t static_value2 = 0xfa2c078ef00d870fULL;
};

template<class ContainerAllocator>
struct DataType< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rtabmap_ros/ListLabelsResponse";
  }

  static const char* value(const ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
string[] labels\n\
";
  }

  static const char* value(const ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.labels);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ListLabelsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rtabmap_ros::ListLabelsResponse_<ContainerAllocator>& v)
  {
    s << indent << "labels[]" << std::endl;
    for (size_t i = 0; i < v.labels.size(); ++i)
    {
      s << indent << "  labels[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.labels[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // RTABMAP_ROS_MESSAGE_LISTLABELSRESPONSE_H
