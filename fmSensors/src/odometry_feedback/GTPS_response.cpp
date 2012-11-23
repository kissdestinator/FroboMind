//=======================================================================
// Basic C++: class GTPS_response
//      Specifications of class GTPS_response
//-----------------------------------------------------------------------
// Auzias Maël - Constantina Ioannou
// For the project: http://bit.ly/kiss-web
//=======================================================================

/*!
 * \file GTPS_response.cpp
 *
 * \brief Implementation of class GTPS_response.
 */

#include "GTPS_response.h"

using namespace std;

/*!
 * Update the last position
 * and the last update time
 */
void GTPS_response::update(boost::shared_ptr<fmMsgs::gtps_<std::allocator<void> > const> const& msg)
{
  _position.set(msg->x, msg->y);
  _last_update = msg->time;
}

/*!
 * Send the response at the service's client
 */
void GTPS_response::response const()
{
  
}