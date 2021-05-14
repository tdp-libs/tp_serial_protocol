#ifndef tp_serial_protocol_SerialMessage_h
#define tp_serial_protocol_SerialMessage_h

#include "tp_serial_protocol/Globals.h"

namespace tp_serial_protocol
{

//##################################################################################################
struct TP_SERIAL_PROTOCOL_SHARED_EXPORT SerialMessage
{
  char command;
  std::string data;

  SerialMessage() = default;

  SerialMessage(char command_, const std::string& data_):
    command(command_),
    data(data_)
  {

  }
};

//##################################################################################################
//! Escape a binary string.
std::string escape(const std::string& input);

//##################################################################################################
//! Unescape an escaped binary string.
std::string unescape(const std::string& input);

//##################################################################################################
//! Escape a binary message.
std::string escapePacket(const SerialMessage& message);

//##################################################################################################
class PartialPacket
{
  int_fast8_t m_gotStart{false};
  int_fast32_t m_index{0};
  int_fast32_t m_bufferSize;
  char* m_commandBuffer;
  std::vector<SerialMessage> m_messages;
public:
  //################################################################################################
  PartialPacket(int bufferSize);

  //################################################################################################
  ~PartialPacket();

  //################################################################################################
  void addData(const std::string& input);

  //################################################################################################
  bool hasMessage() const;

  //################################################################################################
  std::vector<SerialMessage> takeMessages();
};
}

#endif
