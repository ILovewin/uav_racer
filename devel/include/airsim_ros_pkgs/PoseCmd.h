// Generated by gencpp from file airsim_ros_pkgs/PoseCmd.msg
// DO NOT EDIT!


#ifndef AIRSIM_ROS_PKGS_MESSAGE_POSECMD_H
#define AIRSIM_ROS_PKGS_MESSAGE_POSECMD_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace airsim_ros_pkgs
{
template <class ContainerAllocator>
struct PoseCmd_
{
  typedef PoseCmd_<ContainerAllocator> Type;

  PoseCmd_()
    : roll(0.0)
    , pitch(0.0)
    , yaw(0.0)
    , throttle(0.0)  {
    }
  PoseCmd_(const ContainerAllocator& _alloc)
    : roll(0.0)
    , pitch(0.0)
    , yaw(0.0)
    , throttle(0.0)  {
  (void)_alloc;
    }



   typedef double _roll_type;
  _roll_type roll;

   typedef double _pitch_type;
  _pitch_type pitch;

   typedef double _yaw_type;
  _yaw_type yaw;

   typedef double _throttle_type;
  _throttle_type throttle;





  typedef boost::shared_ptr< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> const> ConstPtr;

}; // struct PoseCmd_

typedef ::airsim_ros_pkgs::PoseCmd_<std::allocator<void> > PoseCmd;

typedef boost::shared_ptr< ::airsim_ros_pkgs::PoseCmd > PoseCmdPtr;
typedef boost::shared_ptr< ::airsim_ros_pkgs::PoseCmd const> PoseCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator1> & lhs, const ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator2> & rhs)
{
  return lhs.roll == rhs.roll &&
    lhs.pitch == rhs.pitch &&
    lhs.yaw == rhs.yaw &&
    lhs.throttle == rhs.throttle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator1> & lhs, const ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace airsim_ros_pkgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e72209c904ce31f554c4e432ab36c226";
  }

  static const char* value(const ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe72209c904ce31f5ULL;
  static const uint64_t static_value2 = 0x54c4e432ab36c226ULL;
};

template<class ContainerAllocator>
struct DataType< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "airsim_ros_pkgs/PoseCmd";
  }

  static const char* value(const ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 roll\n"
"float64 pitch\n"
"float64 yaw\n"
"float64 throttle\n"
;
  }

  static const char* value(const ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.roll);
      stream.next(m.pitch);
      stream.next(m.yaw);
      stream.next(m.throttle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PoseCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::airsim_ros_pkgs::PoseCmd_<ContainerAllocator>& v)
  {
    s << indent << "roll: ";
    Printer<double>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<double>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<double>::stream(s, indent + "  ", v.yaw);
    s << indent << "throttle: ";
    Printer<double>::stream(s, indent + "  ", v.throttle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AIRSIM_ROS_PKGS_MESSAGE_POSECMD_H
