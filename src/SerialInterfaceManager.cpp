#include "tp_serial_protocol/SerialInterfaceManager.h"
#include "tp_serial_protocol/SerialInterface.h"

#include "tp_serial_protocol/SerialMessage.h"
#include "tp_serial_protocol/Discovery.h"

#include "tp_utils/DebugUtils.h"
#include "tp_utils/MutexUtils.h"
#include "tp_utils/TimerThread.h"

#include <unordered_set>

namespace tp_serial_protocol
{
//##################################################################################################
struct DeviceDetails_lt
{
  SerialInterfaceManager* q;

  tp_serial_protocol::PortDetails port;

  SerialInterface* serialInterface;

  //################################################################################################
  DeviceDetails_lt():
    q(nullptr),
    serialInterface(nullptr)
  {

  }

  //################################################################################################
  DeviceDetails_lt(SerialInterfaceManager* q_, const tp_serial_protocol::PortDetails& port_):
    q(q_),
    port(port_),
    serialInterface(nullptr)
  {

  }

  //################################################################################################
  ~DeviceDetails_lt()
  {
    delete serialInterface;
  }
};

//##################################################################################################
struct SerialInterfaceManager::Private
{
  SerialInterfaceManager* q;

  std::string deviceType;
  tp_utils::AbstractCrossThreadCallbackFactory* crossThreadCallbackFactory;
  std::vector<std::string> blackList;
  std::vector<std::string> whiteList;

  std::vector<DeviceDetails_lt*> devices;

  tp_utils::TimerThread* timer{nullptr};
  std::unique_ptr<tp_utils::AbstractCrossThreadCallback> timerCallback;
  tp_serial_protocol::Discovery discovery;

  TPMutex configurePort_mutex{TPM};
  std::vector<std::pair<void (*)(void*, serial::Serial&, const PortDetails&), void*>> configurePortCallbacks;

  //################################################################################################
  Private(SerialInterfaceManager* q_,
          const std::string& deviceType_,
          tp_utils::AbstractCrossThreadCallbackFactory* crossThreadCallbackFactory_,
          const std::vector<std::string>& blackList_,
          const std::vector<std::string>& whiteList_):
    q(q_),
    deviceType(deviceType_),
    crossThreadCallbackFactory(crossThreadCallbackFactory_),
    blackList(blackList_),
    whiteList(whiteList_)
  {
    timerCallback.reset(crossThreadCallbackFactory->produce([=]{poll();}));
  }

  //################################################################################################
  void poll()
  {
    bool changed=false;

    //Holds the indexes that we found
    std::unordered_set<size_t> validIndexes;

    //-- Create DeviceDetails_lt for each port ----------------------------------------------------
    discovery.detect();
    for(const tp_serial_protocol::PortDetails& port : discovery.detectedPorts())
    {
      bool found = false;

      if(tpContains(blackList, port.portString))
        continue;

      if(!whiteList.empty() && !tpContains(whiteList, port.portString))
        continue;

      size_t fMax = devices.size();
      for(size_t f=0; f<fMax; f++)
      {
        const DeviceDetails_lt* device = devices.at(f);

        if(device->port.path == port.path)
        {
          validIndexes.insert(f);
          found = true;
          break;
        }
      }

      if(!found)
      {
        changed=true;
        validIndexes.insert(devices.size());
        devices.push_back(new DeviceDetails_lt(q, port));
      }
    }


    //-- Delete obsolete DeviceDetails_lt ---------------------------------------------------------
    for(size_t f=devices.size()-1; f<devices.size(); f--)
    {
      if(!tpContains(validIndexes, f))
      {
        changed = true;
        delete tpTakeAt(devices, f);
      }
    }


    //-- Maintain connections ----------------------------------------------------------------------
    size_t fMax = devices.size();
    for(size_t f=0; f<fMax; f++)
    {
      DeviceDetails_lt* device = devices[f];

      //-- Delete broken connections ---------------------------------------------------------------
      if(device->serialInterface && !device->serialInterface->connected())
      {
        delete device->serialInterface;
        device->serialInterface=nullptr;
      }


      //-- Connect if we are not already -----------------------------------------------------------
      if(!device->serialInterface)
      {
        device->serialInterface = new SerialInterface(device->port, crossThreadCallbackFactory, configurePortCallback);
        device->serialInterface->messageReceived.addCallback([=](const SerialMessage& message){messageReceivedCallback(message, device);});
      }

      //-- Interrogate the device for its type -----------------------------------------------------
      if(device->serialInterface && device->serialInterface->connected())
      {
        device->serialInterface->sendMessage(SerialMessage('t', std::string()));
      }
    }


    //-- Notify changes ----------------------------------------------------------------------------
    if(changed)
      q->managedPortsChanged();
  }

  //################################################################################################
  void messageReceivedCallback(const SerialMessage& message, DeviceDetails_lt* device)
  {
    //-- 't' Type identification request -----------------------------------------------------------
    if(message.command == 't')
    {
      if(device->serialInterface)
        device->serialInterface->sendMessage(SerialMessage('T', deviceType));
    }


    //-- 'T' Type identification response ----------------------------------------------------------
    else if(message.command == 'T' && device->serialInterface)
    {
      if(device->port.deviceType != message.data)
      {
        device->port.deviceType = message.data;

        //Notify changes
        q->managedPortsChanged();
      }
    }


    //-- Call the callback -------------------------------------------------------------------------
    q->messageReceived(message, device->port);
  }

  //################################################################################################
  std::function<void(serial::Serial&, const PortDetails&)> configurePortCallback = [&](serial::Serial& serialPort, const PortDetails& port)
  {
    configurePort_mutex.locked(TPMc [&]
    {
      for(const auto& details : configurePortCallbacks)
        details.first(details.second, serialPort, port);
    });
  };
};

//##################################################################################################
SerialInterfaceManager::SerialInterfaceManager(const std::string& deviceType,
                                               tp_utils::AbstractCrossThreadCallbackFactory* crossThreadCallbackFactory,
                                               const std::vector<std::string>& blackList,
                                               const std::vector<std::string>& whiteList):
  d(new Private(this, deviceType, crossThreadCallbackFactory, blackList, whiteList))
{

}

//##################################################################################################
SerialInterfaceManager::~SerialInterfaceManager()
{
  if(!d->configurePortCallbacks.empty())
    tpWarning() << "Error! SerialInterfaceManager::~SerialInterfaceManager configurePortCallbacks not empty!";

  delete d;
}

//##################################################################################################
void SerialInterfaceManager::setWhiteList(const std::vector<std::string>& whiteList)
{
  d->whiteList = whiteList;
}

//##################################################################################################
void SerialInterfaceManager::addToBlackList(const std::string& portString)
{
  d->blackList.push_back(portString);
}

//##################################################################################################
void SerialInterfaceManager::start()
{
  if(d->timer)
    return;

  d->timer = new tp_utils::TimerThread(*d->timerCallback->callFunctor(), 5000);
}

//##################################################################################################
std::vector<PortDetails> SerialInterfaceManager::detectedPorts()
{
  return d->discovery.detectedPorts();
}

//##################################################################################################
std::vector<PortDetails> SerialInterfaceManager::managedPorts()
{
  std::vector<PortDetails> managedPorts;

  for(const DeviceDetails_lt* device : d->devices)
    managedPorts.push_back(device->port);

  return managedPorts;
}

//##################################################################################################
bool SerialInterfaceManager::connected(const std::string& path)
{
  for(const DeviceDetails_lt* device : d->devices)
  {
    if(device->port.path == path)
    {
      if(device->serialInterface)
        return device->serialInterface->connected();
      break;
    }
  }

  return false;
}

//##################################################################################################
void SerialInterfaceManager::sendMessage(const std::string& path, const SerialMessage& message)
{
  for(const DeviceDetails_lt* device : d->devices)
  {
    if(device->port.path == path)
    {
      if(device->serialInterface)
        device->serialInterface->sendMessage(message);
      else
        tpWarning() << "SerialInterfaceManager::sendMessage() Missing serial interface!";
      return;
    }
  }

  tpWarning() << "SerialInterfaceManager::sendMessage() Failed to find path!";
}

//##################################################################################################
bool SerialInterfaceManager::commandInReceiveBuffer(const std::string& path, char command)
{
  for(const DeviceDetails_lt* dev : d->devices)
    if(dev->port.path == path)
      return (dev->serialInterface)?dev->serialInterface->commandInReceiveBuffer(command):false;

  return false;
}

//##################################################################################################
size_t SerialInterfaceManager::queueSize(const std::string& path)const
{
  for(const DeviceDetails_lt* device : d->devices)
  {
    if(device->port.path == path)
    {
      if(device->serialInterface)
        return device->serialInterface->queueSize();
      break;
    }
  }

  return 0;
}


//##################################################################################################
void SerialInterfaceManager::addConfigurePortCallback(void (*callback)(void*, serial::Serial& port, const PortDetails&), void* opaque)
{
  d->configurePort_mutex.locked(TPMc [&]{d->configurePortCallbacks.emplace_back(callback, opaque);});
}

//##################################################################################################
void SerialInterfaceManager::removeConfigurePortCallback(void (*callback)(void*, serial::Serial& port, const PortDetails&), void* opaque)
{
  tpRemoveOne(d->configurePortCallbacks, std::pair(callback, opaque));
}

}
