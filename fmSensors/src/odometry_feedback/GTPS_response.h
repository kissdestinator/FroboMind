//=======================================================================
// Basic C++: class GTPS_response
//      Specification of class GTPS_response
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
 * \file GTPS_response.h
 *
 * \brief Definition of class GTPS_response.
 */

#ifndef _RESPONSE_H_
#define _RESPONSE_H_

#include "../../../fmPointsMap/src/navigation/Point.h"

/*!
 * \brief Class GTPS_response.
 *
 * Represent the Response sent
 * by the gtps service.
 */
class GTPS_response
{
private:
  Point _position;//!< last position known
  ros::Time _last_update;//!< ros-time of the last update

public:
  // Constructors
  //! Regular constructor.
  GTPS_response() { }

  // Mutators
  //! Method called at each message publish from gtps topic.
  void update(const fmMsgs::gtps::ConstPtr& msg)
  {
    _position.set(msg->x, msg->y);
    _last_update = msg->header.stamp;
  }

  //! Send the response at the client of this service.
  bool response(fmSensors::GTPS::Request  &req,
            	 fmSensors::GTPS::Response &res)
  {
    res.x = _position.x();
    res.y = _position.y();
    res.date = _last_update;
    return true;
  }
};

#endif
