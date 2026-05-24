#pragma once

#include "src/Atava/Hardware/BusConnector.h"
#include "src/Atava/Defines.h"
#include "src/Atava/Utils/Tools.h"

#include "src/include/arduino/Arduino.h"

#define UARTBUFFER_SIZE 32


class GPSCommunication{
    private:
        BusConnector *busConnector;
        CanBus *canBus;

    public:
        void sendCommand(Component &comp,u_int commandID);
        void readData(Component &comp,bool &newTime ,uint32_t &time, uint16_t &date);
        void setup(Component &comp);
        GPSCommunication(BusConnector &busConnector, CanBus *canBus);
        GPSCommunication();
};