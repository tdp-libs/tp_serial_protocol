#ifndef tp_serial_protocol_SimpleSerialInterfaceManager_h
#define tp_serial_protocol_SimpleSerialInterfaceManager_h

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
//! Handles communication with multiple serial ports
class TP_SERIAL_PROTOCOL_SHARED_EXPORT SimpleSerialInterfaceManager
{
public:
  //################################################################################################
  SimpleSerialInterfaceManager(tp_utils::AbstractCrossThreadCallbackFactory* crossThreadCallbackFactory);

  //################################################################################################
  ~SimpleSerialInterfaceManager();

  //################################################################################################
  //! Set the paths that we want to communicate with
  void setPaths(const std::vector<std::string>& path);

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
  void sendMessage(const std::string& path, const std::string& message);

  //################################################################################################
  //! Number of bytes in the queue
  int queueSize(const std::string& path)const;

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

  tp_utils::CallbackCollection<void(const std::string&, const PortDetails&)> messageReceived;
  tp_utils::CallbackCollection<void()> managedPortsChanged;
  tp_utils::CallbackCollection<void()> detectedPortsChanged;

private:
  struct Private;
  Private* d;
  friend struct Private;
};

}

#endif
