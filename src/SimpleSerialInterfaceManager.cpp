#include "tp_serial_protocol/SimpleSerialInterfaceManager.h"
#include "tp_serial_protocol/SimpleSerialInterface.h"

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
  SimpleSerialInterfaceManager* q;

  tp_serial_protocol::PortDetails port;

  SimpleSerialInterface* serialInterface;

  //################################################################################################
  DeviceDetails_lt():
    q(nullptr),
    serialInterface(nullptr)
  {

  }

  //################################################################################################
  DeviceDetails_lt(SimpleSerialInterfaceManager* q_, const tp_serial_protocol::PortDetails& port_):
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
struct SimpleSerialInterfaceManager::Private
{
  SimpleSerialInterfaceManager* q;

  tp_utils::AbstractCrossThreadCallbackFactory* crossThreadCallbackFactory;
  std::unordered_map<void*, std::vector<std::string>> paths;

  std::vector<DeviceDetails_lt*> devices;

  tp_utils::TimerThread* timer{nullptr};
  std::unique_ptr<tp_utils::AbstractCrossThreadCallback> timerCallback;
  tp_serial_protocol::Discovery discovery;

  TPMutex configurePort_mutex{TPM};
  std::vector<std::pair<void (*)(void*, serial::Serial&, const PortDetails&), void*>> configurePortCallbacks;

  //################################################################################################
  Private(SimpleSerialInterfaceManager* q_,
          tp_utils::AbstractCrossThreadCallbackFactory* crossThreadCallbackFactory_):
    q(q_),
    crossThreadCallbackFactory(crossThreadCallbackFactory_)
  {
    timerCallback.reset(crossThreadCallbackFactory->produce([=]{poll();}));
  }

  //################################################################################################
  bool pathRequested(const std::string& path)
  {
    for(const auto& p : paths)
      if(tpContains(p.second, path))
        return true;
    return false;
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

       if(!pathRequested(port.path))
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
        device->serialInterface = new SimpleSerialInterface(device->port, crossThreadCallbackFactory, configurePortCallback);
        device->serialInterface->lineReceived.addCallback([=](const std::string& line){messageReceivedCallback(line, device);});
      }
    }


    //-- Notify changes ----------------------------------------------------------------------------
    if(changed)
      q->managedPortsChanged();
  }

  //################################################################################################
  void messageReceivedCallback(const std::string& message, DeviceDetails_lt* device)
  {
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

  //################################################################################################
  tp_utils::Callback<void()> listChanged = [&]
  {
    q->detectedPortsChanged();
  };
};

//##################################################################################################
SimpleSerialInterfaceManager::SimpleSerialInterfaceManager(tp_utils::AbstractCrossThreadCallbackFactory* crossThreadCallbackFactory):
  d(new Private(this, crossThreadCallbackFactory))
{
  d->listChanged.connect(d->discovery.listChanged);
}

//##################################################################################################
SimpleSerialInterfaceManager::~SimpleSerialInterfaceManager()
{
  if(!d->configurePortCallbacks.empty())
    tpWarning() << "Error! SimpleSerialInterfaceManager::~SimpleSerialInterfaceManager configurePortCallbacks not empty!";

  delete d;
}

//##################################################################################################
void SimpleSerialInterfaceManager::setPaths(const std::vector<std::string>& paths, void* owner)
{
  d->paths[owner] = paths;
}

//##################################################################################################
void SimpleSerialInterfaceManager::clearPaths(void* owner)
{
  d->paths.erase(owner);
}

//##################################################################################################
void SimpleSerialInterfaceManager::start()
{
  if(d->timer)
    return;

  d->timer = new tp_utils::TimerThread(*d->timerCallback->callFunctor(), 5000);
}

//##################################################################################################
std::vector<PortDetails> SimpleSerialInterfaceManager::detectedPorts()
{
  return d->discovery.detectedPorts();
}

//##################################################################################################
std::vector<PortDetails> SimpleSerialInterfaceManager::managedPorts()
{
  std::vector<PortDetails> managedPorts;

  for(const DeviceDetails_lt* device : d->devices)
    managedPorts.push_back(device->port);

  return managedPorts;
}

//##################################################################################################
bool SimpleSerialInterfaceManager::connected(const std::string& path)
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
void SimpleSerialInterfaceManager::sendMessage(const std::string& path, const std::string& message)
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
size_t SimpleSerialInterfaceManager::queueSize(const std::string& path)const
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
void SimpleSerialInterfaceManager::addConfigurePortCallback(void (*callback)(void*, serial::Serial& port, const PortDetails&), void* opaque)
{
  d->configurePort_mutex.locked(TPMc [&]{d->configurePortCallbacks.emplace_back(callback, opaque);});
}

//##################################################################################################
void SimpleSerialInterfaceManager::removeConfigurePortCallback(void (*callback)(void*, serial::Serial& port, const PortDetails&), void* opaque)
{
  tpRemoveOne(d->configurePortCallbacks, std::pair(callback, opaque));
}

}
