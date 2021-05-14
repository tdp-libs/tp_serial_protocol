#ifndef tp_serial_protocol_PortDetails_h
#define tp_serial_protocol_PortDetails_h

#include "tp_serial_protocol/Globals.h"

namespace tp_serial_protocol
{
//##################################################################################################
//! Holds information about a serial port
/*!
This class is used to describe the USB serial ports that have been discovered.
*/
struct TP_SERIAL_PROTOCOL_SHARED_EXPORT PortDetails
{
  //------------------------------------------------------------------------------------------------
  //! The type of board, guessed from the ID
  std::string boardType;

  //------------------------------------------------------------------------------------------------
  //! The path to the serial device
  std::string path;

  //------------------------------------------------------------------------------------------------
  //! The identification string returned by the driver
  std::string portString;

  //------------------------------------------------------------------------------------------------
  //! The type of the connected device
  /*!
  This information is provided by the device that is connected to the serial port and not the driver
  it will be generated in response to a 't' (type) command. This is not done by the Discovery
  process, however the SerialInterfaceManager does do this.

  Known device types:
   - Programming fixture
   - Control application
   - Arduino flasher
  */
  std::string deviceType;

  //################################################################################################
  PortDetails() = default;

  //################################################################################################
  PortDetails(const std::string& boardType_, const std::string& path_, const std::string& portString_);
};

}

#endif
