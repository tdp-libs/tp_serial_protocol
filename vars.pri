TARGET = tp_serial_protocol
TEMPLATE = lib

DEFINES += TP_SERIAL_PROTOCOL_LIBRARY

SOURCES += src/Discovery.cpp
HEADERS += inc/tp_serial_protocol/Discovery.h

SOURCES += src/PortDetails.cpp
HEADERS += inc/tp_serial_protocol/PortDetails.h

SOURCES += src/SimpleSerialInterface.cpp
HEADERS += inc/tp_serial_protocol/SimpleSerialInterface.h

SOURCES += src/SimpleSerialInterfaceManager.cpp
HEADERS += inc/tp_serial_protocol/SimpleSerialInterfaceManager.h

SOURCES += src/SerialInterface.cpp
HEADERS += inc/tp_serial_protocol/SerialInterface.h

SOURCES += src/SerialInterfaceManager.cpp
HEADERS += inc/tp_serial_protocol/SerialInterfaceManager.h

SOURCES += src/SerialMessage.cpp
HEADERS += inc/tp_serial_protocol/SerialMessage.h

SOURCES += src/Globals.cpp
HEADERS += inc/tp_serial_protocol/Globals.h

