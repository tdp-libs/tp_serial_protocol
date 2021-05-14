#ifndef tp_serial_protocol_Discovery_h
#define tp_serial_protocol_Discovery_h

#include "tp_serial_protocol/Globals.h"
#include "tp_serial_protocol/PortDetails.h"

#include "tp_utils/CallbackCollection.h"

namespace tp_serial_protocol
{
//##################################################################################################
//! A class that finds serial ports with possible
class TP_SERIAL_PROTOCOL_SHARED_EXPORT Discovery
{
public:
  //################################################################################################
  Discovery();

  //################################################################################################
  ~Discovery();

  //################################################################################################
  void detect();

  //################################################################################################
  std::vector<PortDetails> detectedPorts();

  tp_utils::CallbackCollection<void()> listChanged;

private:
  struct Private;
  Private* d;
  friend struct Private;
};

}

#endif
