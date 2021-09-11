#ifndef tp_serial_protocol_SerialInterfaceManager_h
#define tp_serial_protocol_SerialInterfaceManager_h

#include "tp_serial_protocol/Globals.h"
#include "tp_serial_protocol/PortDetails.h"

#include "tp_utils/CallbackCollection.h"
#include "tp_utils/AbstractCrossThreadCallback.h"

namespace serial
{
class Serial;
}

namespace tp_serial_protocol
{
struct SerialMessage;

//##################################################################################################
//! Discovers and manages connections to a number of serial devices
/*!
This is used to manage one or more serial devices, it works by opening connections to all of the
ports that fulfill the black/white list requirements. Once it has a connection to a port it will
interrogate the device for its type.

For lists of commands see:
  * tp_serial_protocol/SerialInterfaceManager.h
  * htl_production_hardware_programming_fixture/FixtureProductionHardware.h
  * htl_arduino_flasher/FlasherAPI.h

Core Messages:
<ul>
  <li>'t' = Type identification request
  <li>'T' = Type identification response
  <li>'x' = Reserved
  <li>'X' = Non error messages (uc_utils::Message::message)
  <li>'y' = Reserved
  <li>'Y' = Error messages (uc_utils::Message::error)
  <li>'z' = Reserved
  <li>'Z' = Reserved
</ul>

*/
class TP_SERIAL_PROTOCOL_SHARED_EXPORT SerialInterfaceManager
{
public:
  //################################################################################################
  /*!
  Setup a manage that will scan for serial connections and try to setup communication with them.

  \warning You must call start once you have configured your callbacks!

  \param deviceType - This is how we identify ourself to other devices.
  \param blackList - A list of portString's that we will not attempt to connect to.
  \param whiteList - If this is not empty, we will only connect to portString's in this list.
  */
  SerialInterfaceManager(const std::string& deviceType,
                         tp_utils::AbstractCrossThreadCallbackFactory* crossThreadCallbackFactory,
                         const std::vector<std::string>& blackList=std::vector<std::string>(),
                         const std::vector<std::string>& whiteList=std::vector<std::string>());

  //################################################################################################
  ~SerialInterfaceManager();

  //################################################################################################
  //! Sets the white list all other ports will be ignored
  /*!
  Here you need to use the PortDetails::portString to white list devices.

  \param whiteList - The list of devices to white list, this replaces the existing list.
  */
  void setWhiteList(const std::vector<std::string>& whiteList);

  //################################################################################################
  //! Adds a port to the black list
  void addToBlackList(const std::string& portString);

  //################################################################################################
  //! Call this once after you have configured your callbacks
  void start();

  //################################################################################################
  //! Ports found by the discovery process
  std::vector<PortDetails> detectedPorts();

  //################################################################################################
  //! Returns a list of ports that this is currently managing connections for.
  std::vector<PortDetails> managedPorts();

  //################################################################################################
  //! Returns true if we are connected to a port
  /*!
  This will return true if we are managing and connected to this port.

  \param path - The path to the device, for example '/dev/ttyUSB0'.
  \return True if this class is managing and connected to this device.
  */
  bool connected(const std::string& path);

  //################################################################################################
  //! Send a message to a device
  /*!
  Send a message out to a connected device, if the connection is dropped the message will be
  discarded.

  \param path - The path to the device, for example '/dev/ttyUSB0'.
  \param message - The message to send.
  */
  void sendMessage(const std::string& path, const SerialMessage& message);

  //################################################################################################
  bool commandInReceiveBuffer(const std::string& path, char command);

  //################################################################################################
  size_t queueSize(const std::string& path)const;

  //################################################################################################
  //! This is called just before a serial port is opened
  /*!
  This is called just be for a serial port is opened, allowing the settings of that port to be
  configured.

  \warning The callback must be thread safe!

  \param callback - The callback that will be called just befor the port is opened
  \param opaque - An opaque pointer that will be passed into the callback.
  */
  void addConfigurePortCallback(void (*callback)(void*, serial::Serial& port, const PortDetails&), void* opaque);

  //################################################################################################
  void removeConfigurePortCallback(void (*callback)(void*, serial::Serial& port, const PortDetails&), void* opaque);

  tp_utils::CallbackCollection<void(const SerialMessage&, const PortDetails&)> messageReceived;
  tp_utils::CallbackCollection<void()> managedPortsChanged;

private:
  struct Private;
  Private* d;
  friend struct Private;
};

}

#endif
