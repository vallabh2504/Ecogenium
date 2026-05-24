#pragma once

#include "src/Atava/Hardware/BusConnector.h"
#include "src/Atava/Defines.h"
#include "src/Atava/Utils/Tools.h"

#include "src/include/arduino/Arduino.h"

#define UARTBUFFER_SIZE 32


class CalypsoCommunication{
    private:
        BusConnector *busConnector;

    public:
        void sendSingleMessage(Component &comp, char* message,int length);
        void readData(Component &comp);
        void setup(Component &comp);
        CalypsoCommunication(BusConnector &busConnector);
        CalypsoCommunication();


};