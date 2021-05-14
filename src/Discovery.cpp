#include "tp_serial_protocol/Discovery.h"

#include "tp_utils/FileUtils.h"
#include "tp_utils/DebugUtils.h"

namespace tp_serial_protocol
{

//##################################################################################################
struct Discovery::Private
{
  Discovery* q;
  std::vector<PortDetails> detectedPorts;

  Private(Discovery* q_):
    q(q_)
  {

  }
};

//##################################################################################################
Discovery::Discovery():
  d(new Private(this))
{
  detect();
}

//##################################################################################################
Discovery::~Discovery()
{
  delete d;
}

//##################################################################################################
//Device strings
/*
Arduino Due
  usb-Arduino__www.arduino.cc__Arduino_Due_Prog._Port_64935343433351506190-if00
  usb-Arduino__www.arduino.cc__Arduino_Due_Prog._Port_75338323535351415171-if00
  usb-Arduino_LLC_Arduino_Due-if00

Arduino Mega 2560
  usb-Arduino__www.arduino.cc__0042_853343437383519030B1-if00

Teensy 3.2
  usb-Teensyduino_USB_Serial_1323160-if00
  usb-Teensyduino_USB_Serial_1322780-if00

FTDI
  usb-FTDI_TTL232R-3V3_FTHF2P5D-if00-port0
  usb-FTDI_TTL232R-3V3_FTHF45KL-if00-port0

Prolific knock off
  usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
*/
void Discovery::detect()
{
  d->detectedPorts.clear();
  for(const std::string& path : tp_utils::listFiles("/dev/serial/by-id/", {}))
  {
    std::string port = tp_utils::fileName(path);

    if(tpContains(port, "Arduino"))
    {      
      if(tpContains(port, "0042"))
        d->detectedPorts.emplace_back("Arduino Mega 2560", path, port);
      else if(tpContains(port, "0043"))
        d->detectedPorts.emplace_back("Arduino Uno", path, port);
      else if(tpContains(port, "Due"))
      {
        if(tpContains(port, "Prog"))
          d->detectedPorts.emplace_back("Arduino Due(Programming Port)", path, port);
        else
          d->detectedPorts.emplace_back("Arduino Due(Native USB Port)", path, port);
      }      
      else
        d->detectedPorts.emplace_back("Arduino Other", path, port);
    }
    else if(tpContains(port, "Teensyduino_USB_Serial"))
      d->detectedPorts.emplace_back("Teensy 3.2", path, port);
    else if(tpContains(port, "FTDI"))
      d->detectedPorts.emplace_back("FTDI", path, port);
    else if(tpContains(port, "Prolific"))
      d->detectedPorts.emplace_back("Prolific", path, port);
    else
      d->detectedPorts.emplace_back("Other", path, port);
  }

  listChanged();
}

//##################################################################################################
std::vector<PortDetails> Discovery::detectedPorts()
{
  return d->detectedPorts;
}

}
