
#include "tp_serial_protocol/PortDetails.h"

namespace tp_serial_protocol
{

//##################################################################################################
PortDetails::PortDetails(const std::string& boardType_,
                         const std::string& path_,
                         const std::string& portString_):
  boardType(boardType_),
  path(path_),
  portString(portString_)
{

}


}
