// Generated by gencpp from file jetbot_mini/Position.msg
// DO NOT EDIT!


#ifndef JETBOT_MINI_MESSAGE_POSITION_H
#define JETBOT_MINI_MESSAGE_POSITION_H


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
struct Position_
{
  typedef Position_<ContainerAllocator> Type;

  Position_()
    : x(0.0)
    , y(0.0)  {
    }
  Position_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::jetbot_mini::Position_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jetbot_mini::Position_<ContainerAllocator> const> ConstPtr;

}; // struct Position_

typedef ::jetbot_mini::Position_<std::allocator<void> > Position;

typedef boost::shared_ptr< ::jetbot_mini::Position > PositionPtr;
typedef boost::shared_ptr< ::jetbot_mini::Position const> PositionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jetbot_mini::Position_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jetbot_mini::Position_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jetbot_mini::Position_<ContainerAllocator1> & lhs, const ::jetbot_mini::Position_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jetbot_mini::Position_<ContainerAllocator1> & lhs, const ::jetbot_mini::Position_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jetbot_mini

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::jetbot_mini::Position_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jetbot_mini::Position_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jetbot_mini::Position_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jetbot_mini::Position_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jetbot_mini::Position_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jetbot_mini::Position_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jetbot_mini::Position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "209f516d3eb691f0663e25cb750d67c1";
  }

  static const char* value(const ::jetbot_mini::Position_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x209f516d3eb691f0ULL;
  static const uint64_t static_value2 = 0x663e25cb750d67c1ULL;
};

template<class ContainerAllocator>
struct DataType< ::jetbot_mini::Position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jetbot_mini/Position";
  }

  static const char* value(const ::jetbot_mini::Position_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jetbot_mini::Position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"\n"
;
  }

  static const char* value(const ::jetbot_mini::Position_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jetbot_mini::Position_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Position_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jetbot_mini::Position_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jetbot_mini::Position_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JETBOT_MINI_MESSAGE_POSITION_H
