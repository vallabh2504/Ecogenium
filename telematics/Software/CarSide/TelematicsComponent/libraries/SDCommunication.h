#pragma once
#include "src/Atava/Tasks/Task.h"
#include "src/Atava/Utils/Tools.h"
#include "src/Atava/Hardware/BusConnector.h"
#include "src/Atava/Defines.h"


#include "RP2040_SD/utility/Sd2PinMap.h"
#include "RP2040_SD/RP2040_SD.h"

class SDCommunication{
    private:
        File dataFile;
        uint32_t fileCount = 0;
        bool isSetup = false;

    public:
        bool setup();
        bool writeCanMessage(char* message, int length);
        void open();
        void close();
        bool isSetupFunc();
        void heartBeat(int time);
};