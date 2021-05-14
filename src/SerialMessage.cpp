#include "tp_serial_protocol/SerialMessage.h"

#include "tp_utils/DebugUtils.h"

namespace tp_serial_protocol
{

//##################################################################################################
std::string escape(const std::string& input)
{
  //Calculate the destination size
  int destinationSize = 0;
  {
    const char* i = input.data();
    const char* iMax = i + input.size();

    while(i<iMax)
    {
      switch(*i)
      {
      case '*':
      case '#':
      case '@':
        destinationSize+=2;
        break;
      default:
        destinationSize++;
        break;
      }
      i++;
    }
  }

  std::string output;
  output.resize(destinationSize);
  {
    const char* i = input.data();
    const char* iMax = i + input.size();

    char* o = output.data();

    while(i<iMax)
    {
      if((*i)=='*')
      {
        (*o)='*';
        o++;
        (*o)='0';
      }
      else if((*i)=='#')
      {
        (*o)='*';
        o++;
        (*o)='1';
      }
      else if((*i)=='@')
      {
        (*o)='*';
        o++;
        (*o)='2';
      }
      else
        (*o)=(*i);

      o++;
      i++;
    }
  }

  return output;
}

//##################################################################################################
std::string unescape(const std::string& input)
{
  std::string output;
  output.resize(input.size());

  const char* i = input.data();
  char* o = output.data();

  const char* iMax = i + input.size();

  while(i<iMax)
  {
    if((*i)=='*')
    {
      i++;
      if((*i)=='0')
        (*o)='*';
      else if((*i)=='1')
        (*o)='#';
      else
        (*o)='@';
    }
    else
      (*o)=(*i);

    o++;
    i++;
  }

  output.resize(size_t(o-output.data()));
  return output;
}

//##################################################################################################
std::string escapePacket(const SerialMessage& message)
{
  std::string data;
  data += '@';
  data += message.command;
  data += escape(message.data);
  data += '#';
  return data;
}

//##################################################################################################
PartialPacket::PartialPacket(int bufferSize):
  m_bufferSize(bufferSize),
  m_commandBuffer(new char[bufferSize])
{

}

//##################################################################################################
PartialPacket::~PartialPacket()
{
  delete[] m_commandBuffer;
}

//##################################################################################################
void PartialPacket::addData(const std::string& input)
{
  const char* c = input.data();
  const char* cMax = c+input.size();

  for(; c<cMax; c++)
  {
    int_fast8_t character = *c;
    if(!m_gotStart)
    {
      if(character == '@')
        m_gotStart = true;
    }
    else
    {
      if(character == '#')
      {
        //We should now have a complete message
        if(m_index>=1)
        {
          char command = m_commandBuffer[0];
          std::string data = unescape(std::string(m_commandBuffer+1, size_t(m_index-1)));

          m_messages.emplace_back(command, data);
        }
        m_index = 0;
        m_gotStart = false;
      }
      else
      {
        if(m_index>=m_bufferSize)
        {
          tpWarning() << "Warning! PartialPacket::addChar() Overflow!";
          m_index = 0;
          m_gotStart = false;
        }
        else
        {
          m_commandBuffer[m_index] = character;
          m_index++;
        }
      }
    }
  }
}

//##################################################################################################
bool PartialPacket::hasMessage() const
{
  return !m_messages.empty();
}

//##################################################################################################
std::vector<SerialMessage> PartialPacket::takeMessages()
{
  std::vector<SerialMessage> messages;
  messages.swap(m_messages);
  return messages;
}

}
