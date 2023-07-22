// Generated by gencpp from file towr_ros/TowrCommand.msg
// DO NOT EDIT!


#ifndef TOWR_ROS_MESSAGE_TOWRCOMMAND_H
#define TOWR_ROS_MESSAGE_TOWRCOMMAND_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <xpp_msgs/StateLin3d.h>
#include <xpp_msgs/StateLin3d.h>

namespace towr_ros
{
template <class ContainerAllocator>
struct TowrCommand_
{
  typedef TowrCommand_<ContainerAllocator> Type;

  TowrCommand_()
    : goal_lin()
    , goal_ang()
    , total_duration(0.0)
    , replay_trajectory(false)
    , play_initialization(false)
    , plot_trajectory(false)
    , replay_speed(0.0)
    , optimize(false)
    , robot(0)
    , terrain(0)
    , gait(0)
    , optimize_phase_durations(false)  {
    }
  TowrCommand_(const ContainerAllocator& _alloc)
    : goal_lin(_alloc)
    , goal_ang(_alloc)
    , total_duration(0.0)
    , replay_trajectory(false)
    , play_initialization(false)
    , plot_trajectory(false)
    , replay_speed(0.0)
    , optimize(false)
    , robot(0)
    , terrain(0)
    , gait(0)
    , optimize_phase_durations(false)  {
  (void)_alloc;
    }



   typedef  ::xpp_msgs::StateLin3d_<ContainerAllocator>  _goal_lin_type;
  _goal_lin_type goal_lin;

   typedef  ::xpp_msgs::StateLin3d_<ContainerAllocator>  _goal_ang_type;
  _goal_ang_type goal_ang;

   typedef double _total_duration_type;
  _total_duration_type total_duration;

   typedef uint8_t _replay_trajectory_type;
  _replay_trajectory_type replay_trajectory;

   typedef uint8_t _play_initialization_type;
  _play_initialization_type play_initialization;

   typedef uint8_t _plot_trajectory_type;
  _plot_trajectory_type plot_trajectory;

   typedef double _replay_speed_type;
  _replay_speed_type replay_speed;

   typedef uint8_t _optimize_type;
  _optimize_type optimize;

   typedef int32_t _robot_type;
  _robot_type robot;

   typedef int32_t _terrain_type;
  _terrain_type terrain;

   typedef int32_t _gait_type;
  _gait_type gait;

   typedef uint8_t _optimize_phase_durations_type;
  _optimize_phase_durations_type optimize_phase_durations;





  typedef boost::shared_ptr< ::towr_ros::TowrCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::towr_ros::TowrCommand_<ContainerAllocator> const> ConstPtr;

}; // struct TowrCommand_

typedef ::towr_ros::TowrCommand_<std::allocator<void> > TowrCommand;

typedef boost::shared_ptr< ::towr_ros::TowrCommand > TowrCommandPtr;
typedef boost::shared_ptr< ::towr_ros::TowrCommand const> TowrCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::towr_ros::TowrCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::towr_ros::TowrCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::towr_ros::TowrCommand_<ContainerAllocator1> & lhs, const ::towr_ros::TowrCommand_<ContainerAllocator2> & rhs)
{
  return lhs.goal_lin == rhs.goal_lin &&
    lhs.goal_ang == rhs.goal_ang &&
    lhs.total_duration == rhs.total_duration &&
    lhs.replay_trajectory == rhs.replay_trajectory &&
    lhs.play_initialization == rhs.play_initialization &&
    lhs.plot_trajectory == rhs.plot_trajectory &&
    lhs.replay_speed == rhs.replay_speed &&
    lhs.optimize == rhs.optimize &&
    lhs.robot == rhs.robot &&
    lhs.terrain == rhs.terrain &&
    lhs.gait == rhs.gait &&
    lhs.optimize_phase_durations == rhs.optimize_phase_durations;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::towr_ros::TowrCommand_<ContainerAllocator1> & lhs, const ::towr_ros::TowrCommand_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace towr_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::towr_ros::TowrCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::towr_ros::TowrCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::towr_ros::TowrCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::towr_ros::TowrCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::towr_ros::TowrCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::towr_ros::TowrCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::towr_ros::TowrCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9687aa81cfa0759f837eb5ec3b2f880e";
  }

  static const char* value(const ::towr_ros::TowrCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9687aa81cfa0759fULL;
  static const uint64_t static_value2 = 0x837eb5ec3b2f880eULL;
};

template<class ContainerAllocator>
struct DataType< ::towr_ros::TowrCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "towr_ros/TowrCommand";
  }

  static const char* value(const ::towr_ros::TowrCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::towr_ros::TowrCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# The command for the robot specified by the user\n"
"\n"
"xpp_msgs/StateLin3d    goal_lin                  # the linear state to reach (x,y,z and derivatives)\n"
"xpp_msgs/StateLin3d    goal_ang                  # the angular state to reach (roll, pitch, yaw and derivatives)\n"
"float64                total_duration            # the total time to reach the goal state\n"
"bool                   replay_trajectory         # Replay the already optimized trajectory in RVIZ\n"
"bool                   play_initialization       # Play motion generated by unoptimized initial variables\n"
"bool                   plot_trajectory           # Plot the optimized trajectory\n"
"float64                replay_speed              # speed at which to playback the motion.\n"
"bool                   optimize                  # run TOWR optimization\n"
"int32                  robot                     # Monoped, Biped, Quadruped, Anymal\n"
"int32                  terrain                   # some information about the used terrain (e.g stairs, gap, slope)\n"
"int32                  gait                      # Type of Motion (Walk, Trott, Bound, Pace)\n"
"bool                   optimize_phase_durations  # If true, the gait is optimized over as well\n"
"\n"
"================================================================================\n"
"MSG: xpp_msgs/StateLin3d\n"
"# This contains the 3D representation of a linear state, including:\n"
"# position, velocity, acceleration\n"
"\n"
"geometry_msgs/Point pos\n"
"geometry_msgs/Vector3 vel\n"
"geometry_msgs/Vector3 acc\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::towr_ros::TowrCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::towr_ros::TowrCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.goal_lin);
      stream.next(m.goal_ang);
      stream.next(m.total_duration);
      stream.next(m.replay_trajectory);
      stream.next(m.play_initialization);
      stream.next(m.plot_trajectory);
      stream.next(m.replay_speed);
      stream.next(m.optimize);
      stream.next(m.robot);
      stream.next(m.terrain);
      stream.next(m.gait);
      stream.next(m.optimize_phase_durations);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TowrCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::towr_ros::TowrCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::towr_ros::TowrCommand_<ContainerAllocator>& v)
  {
    s << indent << "goal_lin: ";
    s << std::endl;
    Printer< ::xpp_msgs::StateLin3d_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_lin);
    s << indent << "goal_ang: ";
    s << std::endl;
    Printer< ::xpp_msgs::StateLin3d_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_ang);
    s << indent << "total_duration: ";
    Printer<double>::stream(s, indent + "  ", v.total_duration);
    s << indent << "replay_trajectory: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.replay_trajectory);
    s << indent << "play_initialization: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.play_initialization);
    s << indent << "plot_trajectory: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.plot_trajectory);
    s << indent << "replay_speed: ";
    Printer<double>::stream(s, indent + "  ", v.replay_speed);
    s << indent << "optimize: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.optimize);
    s << indent << "robot: ";
    Printer<int32_t>::stream(s, indent + "  ", v.robot);
    s << indent << "terrain: ";
    Printer<int32_t>::stream(s, indent + "  ", v.terrain);
    s << indent << "gait: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gait);
    s << indent << "optimize_phase_durations: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.optimize_phase_durations);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TOWR_ROS_MESSAGE_TOWRCOMMAND_H
