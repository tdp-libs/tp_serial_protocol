#ifndef tp_serial_protocol_SimpleSerialInterface_h
#define tp_serial_protocol_SimpleSerialInterface_h

#include "tp_serial_protocol/Globals.h"

#include "tp_utils/CallbackCollection.h"
#include "tp_utils/AbstractCrossThreadCallback.h"

namespace serial
{
class Serial;
}

namespace tp_serial_protocol
{
struct PortDetails;
struct SerialMessage;

//##################################################################################################
class TP_SERIAL_PROTOCOL_SHARED_EXPORT SimpleSerialInterface
{
public:
  //################################################################################################
  /*!
  \param port
  \param crossThreadCallbackFactory
  \param configurePort_callback will be called in a thread to configure the serial port.
  */
  SimpleSerialInterface(const PortDetails& port,
                        tp_utils::AbstractCrossThreadCallbackFactory* crossThreadCallbackFactory,
                        const std::function<void(serial::Serial&, const PortDetails&)>& configurePort_callback=configurePortCallback);

  //################################################################################################
  ~SimpleSerialInterface();

  //################################################################################################
  bool connected();

  //################################################################################################
  void sendMessage(const std::string& message);

  //################################################################################################
  //! Number of bytes to send
  size_t queueSize()const;

  //################################################################################################
  //! Called each time a message comes in.
  tp_utils::CallbackCollection<void(const std::string&)> lineReceived;

  //################################################################################################
  static void configurePortCallback(serial::Serial& serialPort, const PortDetails& port);

private:
  class Private;
  Private* d;
  friend class Private;
};

}

#endif
