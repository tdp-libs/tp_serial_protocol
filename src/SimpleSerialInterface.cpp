#include "tp_serial_protocol/SimpleSerialInterface.h"
#include "tp_serial_protocol/SerialMessage.h"
#include "tp_serial_protocol/PortDetails.h"

#include "tp_utils/MutexUtils.h"
#include "tp_utils/DebugUtils.h"

#include "serial/serial.h"

#include <iostream>
#include <thread>

namespace tp_serial_protocol
{

namespace
{
//The size of the buffer to allocate for received messages
size_t maxBufferSize = 1048576;
}

//##################################################################################################
class SimpleSerialInterface::Private
{
public:
  SimpleSerialInterface* q;
  PortDetails port;

  TPMutex mutex{TPM};
  bool connected{false};
  bool finish{false};

  TPMutex messageQueueMutex{TPM};
  std::vector<std::string> messageQueue;

  TPMutex receivedMessageMutex{TPM};
  std::vector<std::string> receivedMessages;

  const std::function<void(serial::Serial&, const PortDetails&)>& configurePort_callback;

  std::unique_ptr<tp_utils::AbstractCrossThreadCallback> messagesReceivedCallback;

  std::thread thread;

  //################################################################################################
  Private(SimpleSerialInterface* q_, const PortDetails& port_,
          tp_utils::AbstractCrossThreadCallbackFactory* crossThreadCallbackFactory,
          const std::function<void(serial::Serial&, const PortDetails&)>& configurePort_callback_):
    q(q_),
    port(port_),
    configurePort_callback(configurePort_callback_),
    thread([this]{run();})
  {
    messagesReceivedCallback.reset(crossThreadCallbackFactory->produce([&]{messagesReceived();}));
  }

  //################################################################################################
  ~Private()
  {
    mutex.locked(TPMc [&]{finish = true;});
    thread.join();
  }

  // //################################################################################################
  // QString serialPortError(QSerialPort::SerialPortError code)
  // {
  //   switch(code)
  //   {
  //   case QSerialPort::NoError:                   return "NoError";
  //   case QSerialPort::DeviceNotFoundError:       return "DeviceNotFoundError";
  //   case QSerialPort::PermissionError:           return "PermissionError";
  //   case QSerialPort::OpenError:                 return "OpenError";
  //   case QSerialPort::ParityError:               return "ParityError";
  //   case QSerialPort::FramingError:              return "FramingError";
  //   case QSerialPort::BreakConditionError:       return "BreakConditionError";
  //   case QSerialPort::WriteError:                return "WriteError";
  //   case QSerialPort::ReadError:                 return "ReadError";
  //   case QSerialPort::ResourceError:             return "ResourceError";
  //   case QSerialPort::UnsupportedOperationError: return "UnsupportedOperationError";
  //   case QSerialPort::UnknownError:              return "UnknownError";
  //   case QSerialPort::TimeoutError:              return "TimeoutError";
  //   case QSerialPort::NotOpenError:              return "NotOpenError";
  //   }
  //   return "...";
  // }

  //################################################################################################
  void run()
  {
    TP_CLEANUP([&]{mutex.locked(TPMc [&]{connected = false;});});

    serial::Serial serialPort;

    try
    {
      serialPort.setPort(port.path);
    }
    catch(const std::invalid_argument&)
    {
      tpWarning() << "Error serialPort.setPort() std::invalid_argument, path: " << port.path;
      return;
    }

    if(configurePort_callback)
      configurePort_callback(serialPort, port);

    try
    {
      serialPort.open();
    }
    catch(const std::invalid_argument&)
    {
      tpWarning() << "Error serialPort.open() std::invalid_argument, path: " << port.path;
      return;
    }
    catch(const serial::SerialException&)
    {
      tpWarning() << "Error serialPort.open() serial::SerialException, path: " << port.path;
      return;
    }
    catch(const serial::IOException&)
    {
      tpWarning() << "Error serialPort.open() serial::IOException, path: " << port.path;
      return;
    }

    try
    {
      serialPort.setDTR(true);
    }
    catch(const serial::PortNotOpenedException&)
    {
      tpWarning() << "Error serialPort.open() serial::PortNotOpenedException, path: " << port.path;
      return;
    }
    catch(const serial::SerialException&)
    {
      tpWarning() << "Error serialPort.open() serial::SerialException, path: " << port.path;
      return;
    }

    mutex.locked(TPMc [&]{connected = true;});

    std::string buffer;

    serialPort.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);

    mutex.lock(TPM);
    while(!finish && serialPort.isOpen())
    {
      mutex.unlock(TPM);

      size_t a{0};
      try
      {
        a = serialPort.available();
      }
      catch(const serial::IOException&)
      {
        tpWarning() << "Error serialPort.available() serial::IOException, path: " << port.path;
        mutex.lock(TPM);
        break;
      }

      if(a>0)
      {
        try
        {
          buffer += serialPort.read(std::min(size_t(1024), a));
        }
        catch(const serial::PortNotOpenedException&)
        {
          tpWarning() << "Error serialPort.read() serial::PortNotOpenedException, path: " << port.path;
          mutex.lock(TPM);
          break;
        }
        catch(const serial::SerialException&)
        {
          tpWarning() << "Error serialPort.read() serial::PortNotOpenedException, path: " << port.path;
          mutex.lock(TPM);
          break;
        }

        for(auto i=buffer.find_first_of('\n'); i!=std::string::npos; i=buffer.find_first_of('\n'))
        {
          receivedMessageMutex.locked(TPMc [&]
          {
            receivedMessages.push_back(buffer.substr(0, i));
            buffer.erase(0, i+1);
          });

          messagesReceivedCallback->call();
        }

        if(buffer.size()>maxBufferSize)
          buffer.clear();
      }
      else
      {
        //Is there anything to send, if so send it
        serialPort.flush();

        bool failed=false;
        messageQueueMutex.locked(TPMc [&]
        {
          if(!messageQueue.empty())
          {
            try
            {
              serialPort.write(tpTakeFirst(messageQueue));
            }
            catch(const serial::PortNotOpenedException&)
            {
              tpWarning() << "Error serialPort.write() serial::PortNotOpenedException, path: " << port.path;
              failed = true;
              return;
            }
            catch(const serial::SerialException&)
            {
              tpWarning() << "Error serialPort.write() serial::PortNotOpenedException, path: " << port.path;
              failed = true;
              return;
            }
            catch(const serial::IOException&)
            {
              tpWarning() << "Error serialPort.write() serial::IOException, path: " << port.path;
              failed = true;
              return;
            }
          }
        });

        if(failed)
        {
          mutex.lock(TPM);
          break;
        }
      }

      mutex.lock(TPM);
    }
    mutex.unlock(TPM);
  }

  //################################################################################################
  void messagesReceived()
  {
    std::vector<std::string> messages;
    receivedMessageMutex.locked(TPMc [&]{messages.swap(receivedMessages);});
    for(const auto& message : messages)
      q->lineReceived(message);
  }
};

//##################################################################################################
SimpleSerialInterface::SimpleSerialInterface(const PortDetails& port,
                                 tp_utils::AbstractCrossThreadCallbackFactory* crossThreadCallbackFactory,
                                 const std::function<void(serial::Serial&, const PortDetails&)>& configurePort_callback):
  d(new Private(this, port, crossThreadCallbackFactory, configurePort_callback))
{

}

//##################################################################################################
SimpleSerialInterface::~SimpleSerialInterface()
{
  delete d;
}

//##################################################################################################
bool SimpleSerialInterface::connected()
{
  TP_MUTEX_LOCKER(d->mutex);
  return d->connected;
}

//##################################################################################################
void SimpleSerialInterface::sendMessage(const std::string& message)
{
  TP_MUTEX_LOCKER(d->messageQueueMutex);
  d->messageQueue.push_back(message);
}

//##################################################################################################
size_t SimpleSerialInterface::queueSize()const
{
  TP_MUTEX_LOCKER(d->messageQueueMutex);
  return d->messageQueue.size();
}

//##################################################################################################
void SimpleSerialInterface::configurePortCallback(serial::Serial& serialPort, const PortDetails& port)
{
  //The Arduino Due native USB does not need any port configuration, however for the rest of the
  //devices we need to configure a baud rate.
  if(port.boardType != "Arduino Due(Native USB Port)" &&
     port.boardType != "Teensy 3.2")
  {
    try
    {
      serialPort.setBaudrate(57600);
    }
    catch(const std::invalid_argument&)
    {
      tpWarning() << "Error setting baudrate to 57600!";
    }
  }
}

}
