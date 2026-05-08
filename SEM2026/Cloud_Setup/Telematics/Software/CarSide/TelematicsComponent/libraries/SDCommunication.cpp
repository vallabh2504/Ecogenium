#include "SDCommunication.h"

SPIClassRP2040 SPI(spi0, 16, 17, 18, 19);

#include "RP2040_SD/utility/Sd2PinMap.h"
#include "RP2040_SD/RP2040_SD.h"

bool SDCommunication::setup(){
    if (!SD.begin(17)) //Vorher PIN_SD_SS in Klammern
    {
        Tools::log(INFO,"TC", "SDCom", "Setup: Initializing failed!");
        isSetup = false;
        return false;
    }
    Tools::log(INFO,"TC", "SDCom", "Setup: Succeeded!");
    isSetup = true;
    while(SD.exists(std::to_string(fileCount).c_str())){
        fileCount++;
        Tools::log(INFO, "TC", "SDCom", "filecount: %d",fileCount);
    }
    //Tools::log(INFO, "TC", "SDCom", "filename: %s",strcat(strcat("datalog_" ,std::to_string(size + 1).c_str()),".txt"));
    //fileName = strcat(strcat("datalog_" ,std::to_string(size + 1).c_str()),".txt");


    return true;
}

bool SDCommunication::writeCanMessage(char* message, int length){
    if(isSetup){
        // if the file is available, write to it:
        if (dataFile.availableForWrite()){
            dataFile.write(message,length);
            return true;
        }else{    // if the file isn't open send an error:

            Tools::log(INFO, "TC", "SDCom", "Error: Not available for Write");
            return false;
        }
    }
    return false;
}

void SDCommunication::open(){
    if(isSetup){
        Tools::log(INFO, "TC", "SDCom", "Opening!");
        dataFile = SD.open(std::to_string(fileCount).c_str(), FILE_WRITE);
        if(!dataFile.availableForWrite()){
            if(!SD.begin(17)){
                 Tools::log(INFO,"TC", "SDCom", "Setup: Initializing failed!");
            }else{
                setup();
            }
        }
    }
}

void SDCommunication::heartBeat(int time){
    if(isSetup){
        Tools::log(INFO,"TC","SDCom","Heart");
        File heartFile = SD.open("heartBeat", FILE_WRITE);
        delay(1000);
        if(heartFile.availableForWrite()){
            heartFile.write(std::to_string(time).c_str());
            heartFile.flush();
        }
        heartFile.close();
    }   
}


void SDCommunication::close(){
    if(isSetup){
        Tools::log(INFO, "TC", "SDCom", "Closing!");
        dataFile.close();
    }
}

bool SDCommunication::isSetupFunc(){
    return isSetup;
}
