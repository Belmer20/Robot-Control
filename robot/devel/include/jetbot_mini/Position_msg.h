// Generated by gencpp from file jetbot_mini/Position_msg.msg
// DO NOT EDIT!


#ifndef JETBOT_MINI_MESSAGE_POSITION_MSG_H
#define JETBOT_MINI_MESSAGE_POSITION_MSG_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jetbot_mini
{
template <class ContainerAllocator>
struct Position_msg_
{
  typedef Position_msg_<ContainerAllocator> Type;

  Position_msg_()
    : robot_index(0)
    , x(0.0)
    , y(0.0)  {
    }
  Position_msg_(const ContainerAllocator& _alloc)
    : robot_index(0)
    , x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _robot_index_type;
  _robot_index_type robot_index;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::jetbot_mini::Position_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jetbot_mini::Position_msg_<ContainerAllocator> const> ConstPtr;

}; // struct Position_msg_

typedef ::jetbot_mini::Position_msg_<std::allocator<void> > Position_msg;

typedef boost::shared_ptr< ::jetbot_mini::Position_msg > Position_msgPtr;
typedef boost::shared_ptr< ::jetbot_mini::Position_msg const> Position_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jetbot_mini::Position_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jetbot_mini::Position_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jetbot_mini::Position_msg_<ContainerAllocator1> & lhs, const ::jetbot_mini::Position_msg_<ContainerAllocator2> & rhs)
{
  return lhs.robot_index == rhs.robot_index &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jetbot_mini::Position_msg_<ContainerAllocator1> & lhs, const ::jetbot_mini::Position_msg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jetbot_mini

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::jetbot_mini::Position_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jetbot_mini::Position_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jetbot_mini::Position_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jetbot_mini::Position_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jetbot_mini::Position_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jetbot_mini::Position_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jetbot_mini::Position_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c8685de48e63ca6aa63f0d3bc9a2af72";
  }

  static const char* value(const ::jetbot_mini::Position_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc8685de48e63ca6aULL;
  static const uint64_t static_value2 = 0xa63f0d3bc9a2af72ULL;
};

template<class ContainerAllocator>
struct DataType< ::jetbot_mini::Position_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jetbot_mini/Position_msg";
  }

  static const char* value(const ::jetbot_mini::Position_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jetbot_mini::Position_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 robot_index\n"
"float64 x\n"
"float64 y\n"
"\n"
;
  }

  static const char* value(const ::jetbot_mini::Position_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jetbot_mini::Position_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.robot_index);
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Position_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jetbot_mini::Position_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jetbot_mini::Position_msg_<ContainerAllocator>& v)
  {
    s << indent << "robot_index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.robot_index);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JETBOT_MINI_MESSAGE_POSITION_MSG_H
