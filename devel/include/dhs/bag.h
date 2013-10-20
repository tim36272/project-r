/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
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
 *  * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Auto-generated by genmsg_cpp from file /nfs/home/tsweet/workspace/DHS/src/dhs/msg/bag.msg
 *
 */


#ifndef DHS_MESSAGE_BAG_H
#define DHS_MESSAGE_BAG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dhs
{
template <class ContainerAllocator>
struct bag_
{
  typedef bag_<ContainerAllocator> Type;

  bag_()
    : blue(0)
    , green(0)
    , red(0)  {
    }
  bag_(const ContainerAllocator& _alloc)
    : blue(0)
    , green(0)
    , red(0)  {
    }



   typedef uint8_t _blue_type;
  _blue_type blue;

   typedef uint8_t _green_type;
  _green_type green;

   typedef uint8_t _red_type;
  _red_type red;




  typedef boost::shared_ptr< ::dhs::bag_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dhs::bag_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct bag_

typedef ::dhs::bag_<std::allocator<void> > bag;

typedef boost::shared_ptr< ::dhs::bag > bagPtr;
typedef boost::shared_ptr< ::dhs::bag const> bagConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dhs::bag_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dhs::bag_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dhs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg'], 'dhs': ['/nfs/home/tsweet/workspace/DHS/src/dhs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dhs::bag_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dhs::bag_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dhs::bag_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dhs::bag_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dhs::bag_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dhs::bag_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dhs::bag_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ecb8956a75edf997e18aad34e20d469b";
  }

  static const char* value(const ::dhs::bag_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xecb8956a75edf997ULL;
  static const uint64_t static_value2 = 0xe18aad34e20d469bULL;
};

template<class ContainerAllocator>
struct DataType< ::dhs::bag_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dhs/bag";
  }

  static const char* value(const ::dhs::bag_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dhs::bag_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 blue\n\
uint8 green\n\
uint8 red\n\
\n\
";
  }

  static const char* value(const ::dhs::bag_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dhs::bag_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.blue);
      stream.next(m.green);
      stream.next(m.red);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct bag_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dhs::bag_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dhs::bag_<ContainerAllocator>& v)
  {
    s << indent << "blue: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.blue);
    s << indent << "green: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.green);
    s << indent << "red: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.red);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DHS_MESSAGE_BAG_H
