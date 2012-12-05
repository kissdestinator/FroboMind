/* Auto-generated by genmsg_cpp for file /home/con/fuerte_workspace/FroboMind/fmSensors/srv/GTPS.srv */
#ifndef FMSENSORS_SERVICE_GTPS_H
#define FMSENSORS_SERVICE_GTPS_H
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

#include "ros/service_traits.h"




namespace fmSensors
{
template <class ContainerAllocator>
struct GTPSRequest_ {
  typedef GTPSRequest_<ContainerAllocator> Type;

  GTPSRequest_()
  {
  }

  GTPSRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::fmSensors::GTPSRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fmSensors::GTPSRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct GTPSRequest
typedef  ::fmSensors::GTPSRequest_<std::allocator<void> > GTPSRequest;

typedef boost::shared_ptr< ::fmSensors::GTPSRequest> GTPSRequestPtr;
typedef boost::shared_ptr< ::fmSensors::GTPSRequest const> GTPSRequestConstPtr;


template <class ContainerAllocator>
struct GTPSResponse_ {
  typedef GTPSResponse_<ContainerAllocator> Type;

  GTPSResponse_()
  : x(0)
  , y(0)
  , date()
  {
  }

  GTPSResponse_(const ContainerAllocator& _alloc)
  : x(0)
  , y(0)
  , date()
  {
  }

  typedef int32_t _x_type;
  int32_t x;

  typedef int32_t _y_type;
  int32_t y;

  typedef ros::Time _date_type;
  ros::Time date;


  typedef boost::shared_ptr< ::fmSensors::GTPSResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fmSensors::GTPSResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct GTPSResponse
typedef  ::fmSensors::GTPSResponse_<std::allocator<void> > GTPSResponse;

typedef boost::shared_ptr< ::fmSensors::GTPSResponse> GTPSResponsePtr;
typedef boost::shared_ptr< ::fmSensors::GTPSResponse const> GTPSResponseConstPtr;

struct GTPS
{

typedef GTPSRequest Request;
typedef GTPSResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct GTPS
} // namespace fmSensors

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fmSensors::GTPSRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fmSensors::GTPSRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fmSensors::GTPSRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::fmSensors::GTPSRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::fmSensors::GTPSRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmSensors/GTPSRequest";
  }

  static const char* value(const  ::fmSensors::GTPSRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fmSensors::GTPSRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::fmSensors::GTPSRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::fmSensors::GTPSRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fmSensors::GTPSResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fmSensors::GTPSResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fmSensors::GTPSResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8c7b89222eea218aa26cd807453bd8e1";
  }

  static const char* value(const  ::fmSensors::GTPSResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8c7b89222eea218aULL;
  static const uint64_t static_value2 = 0xa26cd807453bd8e1ULL;
};

template<class ContainerAllocator>
struct DataType< ::fmSensors::GTPSResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmSensors/GTPSResponse";
  }

  static const char* value(const  ::fmSensors::GTPSResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fmSensors::GTPSResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 x\n\
int32 y\n\
time date\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::fmSensors::GTPSResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::fmSensors::GTPSResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fmSensors::GTPSRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GTPSRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fmSensors::GTPSResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.date);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GTPSResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<fmSensors::GTPS> {
  static const char* value() 
  {
    return "8c7b89222eea218aa26cd807453bd8e1";
  }

  static const char* value(const fmSensors::GTPS&) { return value(); } 
};

template<>
struct DataType<fmSensors::GTPS> {
  static const char* value() 
  {
    return "fmSensors/GTPS";
  }

  static const char* value(const fmSensors::GTPS&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<fmSensors::GTPSRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8c7b89222eea218aa26cd807453bd8e1";
  }

  static const char* value(const fmSensors::GTPSRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<fmSensors::GTPSRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmSensors/GTPS";
  }

  static const char* value(const fmSensors::GTPSRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<fmSensors::GTPSResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8c7b89222eea218aa26cd807453bd8e1";
  }

  static const char* value(const fmSensors::GTPSResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<fmSensors::GTPSResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fmSensors/GTPS";
  }

  static const char* value(const fmSensors::GTPSResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // FMSENSORS_SERVICE_GTPS_H
