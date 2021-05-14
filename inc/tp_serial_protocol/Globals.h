#ifndef tp_serial_protocol_Globals_h
#define tp_serial_protocol_Globals_h

#include "tp_utils/Globals.h"

#if defined(TP_SERIAL_PROTOCOL_LIBRARY)
#  define TP_SERIAL_PROTOCOL_SHARED_EXPORT TP_EXPORT
#else
#  define TP_SERIAL_PROTOCOL_SHARED_EXPORT TP_IMPORT
#endif

//##################################################################################################
//! Code for communicating with devices over serial links.
/*!

*/
namespace tp_serial_protocol
{

}

#endif
