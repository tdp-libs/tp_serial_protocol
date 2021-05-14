#include "tp_serial_protocol/SerialInterface.h"
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
int bufferSize = 1048576;
}

//##################################################################################################
class SerialInterface::Private
{
public:
  SerialInterface* q;
  PortDetails port;

  TPMutex mutex{TPM};
  bool connected{false};
  bool finish{false};

  TPMutex messageQueueMutex;
  std::vector<SerialMessage> messageQueue;

  TPMutex receivedMessageMutex;
  std::vector<SerialMessage> receivedMessages;

  const std::function<void(serial::Serial&, const PortDetails&)>& configurePort_callback;

  std::unique_ptr<tp_utils::AbstractCrossThreadCallback> messagesReceivedCallback;

  std::thread thread;

  //################################################################################################
  Private(SerialInterface* q_, const PortDetails& port_,
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

    mutex.locked(TPMc [&]{connected = true;});


    PartialPacket partialPacket(bufferSize);



    // // Test the timeout at 250ms
    // my_serial.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);
    // count = 0;
    // cout << "Timeout == 250ms, asking for 1 more byte than written." << endl;
    // while (count < 10) {
    //   size_t bytes_wrote = my_serial.write(test_string);
    //
    //   string result = my_serial.read(test_string.length()+1);
    //
    //   cout << "Iteration: " << count << ", Bytes written: ";
    //   cout << bytes_wrote << ", Bytes read: ";
    //   cout << result.length() << ", String read: " << result << endl;
    //
    //   count += 1;
    // }




    serialPort.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);



    mutex.lock();
    while(!finish && serialPort.isOpen())
    {
      // //We expect to get timeouts thats normal when waiting for data
      // if(serialPort.error() == QSerialPort::TimeoutError)
      //   serialPort.clearError();
      // else if(serialPort.error())
      // {
      //
      //
      //   qWarning() << "tp_qt_utils::SerialInterface, Serial port error: " << serialPort.errorString() <<
      //                 " Error code: " << serialPort.error() <<
      //                 " Serial port error: " << serialPortError(serialPort.error()) <<
      //                 " Port: " << port.path;
      //   break;
      // }

      mutex.unlock();

      size_t a{0};
      try
      {
        a = serialPort.available();
      }
      catch(const serial::IOException&)
      {
        tpWarning() << "Error serialPort.available() serial::IOException, path: " << port.path;
        mutex.lock();
        break;
      }

      if(a>0)
      {
        try
        {
          partialPacket.addData(serialPort.read(std::min(size_t(1024), a)));
        }
        catch(const serial::PortNotOpenedException&)
        {
          tpWarning() << "Error serialPort.read() serial::PortNotOpenedException, path: " << port.path;
          mutex.lock();
          break;
        }
        catch(const serial::SerialException&)
        {
          tpWarning() << "Error serialPort.read() serial::PortNotOpenedException, path: " << port.path;
          mutex.lock();
          break;
        }

        if(partialPacket.hasMessage())
        {
          receivedMessageMutex.locked([&]
          {
            std::vector<SerialMessage> messages = partialPacket.takeMessages();
            receivedMessages.reserve(receivedMessages.size()+messages.size());
            for(const SerialMessage& message : messages)
              receivedMessages.push_back(message);
          });

          messagesReceivedCallback->call();
        }
      }
      else
      {
        //Is there anything to send, if so send it
        serialPort.flush();

        // //Flush can be a blocking operation, if it blocks we will get an error because we have
        // //selected non blocking IO, if this happens ignore the error and allow the loop to continue
        // //until this data is sent.
        // if(serialPort.error() == QSerialPort::ResourceError)
        //   serialPort.clearError();
        //
        // //If flush succeeded send more data if we have something to send.
        // else if(serialPort.bytesToWrite()<1)
        // {
        //   messageQueueMutex.lock();
        //   if(!messageQueue.isEmpty())
        //     serialPort.write(tp_qt_utils::escapePacket(messageQueue.takeFirst()));
        //   messageQueueMutex.unlock();
        // }

        bool failed=false;
        messageQueueMutex.locked([&]
        {
          if(!messageQueue.empty())
          {
            try
            {
              serialPort.write(escapePacket(tpTakeFirst(messageQueue)));
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
          mutex.lock();
          break;
        }
      }

      mutex.lock();
    }
    connected = false;
    mutex.unlock();
  }

  //################################################################################################
  void messagesReceived()
  {
    std::vector<SerialMessage> messages;
    receivedMessageMutex.locked([&]{messages.swap(receivedMessages);});
    for(const auto& message : messages)
      q->messageReceived(message);
  }
};

//##################################################################################################
SerialInterface::SerialInterface(const PortDetails& port,
                                 tp_utils::AbstractCrossThreadCallbackFactory* crossThreadCallbackFactory,
                                 const std::function<void(serial::Serial&, const PortDetails&)>& configurePort_callback):
  d(new Private(this, port, crossThreadCallbackFactory, configurePort_callback))
{

}

//##################################################################################################
SerialInterface::~SerialInterface()
{
  delete d;
}

//##################################################################################################
bool SerialInterface::connected()
{
  TP_MUTEX_LOCKER(d->mutex);
  return d->connected;
}

//##################################################################################################
void SerialInterface::sendMessage(const SerialMessage& message)
{
  TP_MUTEX_LOCKER(d->messageQueueMutex);
  d->messageQueue.push_back(message);
#warning wake here
}

//##################################################################################################
bool SerialInterface::commandInReceiveBuffer(char command)
{
  TP_MUTEX_LOCKER(d->receivedMessageMutex);
  for(const SerialMessage& message : d->receivedMessages)
    if(message.command == command)
      return true;

  return false;
}

//##################################################################################################
int SerialInterface::queueSize()const
{
  TP_MUTEX_LOCKER(d->messageQueueMutex);
  return d->messageQueue.size();
}

//##################################################################################################
void SerialInterface::configurePortCallback(serial::Serial& serialPort, const PortDetails& port)
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
