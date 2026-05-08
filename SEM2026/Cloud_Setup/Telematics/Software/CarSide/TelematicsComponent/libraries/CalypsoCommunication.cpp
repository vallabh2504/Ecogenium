#include "CalypsoCommunication.h"
#include <string>

void CalypsoCommunication::setup(Component &comp){
    busConnector->enableUart(&comp,0);
    busConnector->startReceivingUart(&comp,0,921600);


}


void CalypsoCommunication::readData(Component &comp){
    char* output = (char*)malloc(100);
    uint8_t readCount = 0;
    //Read as long as there is uart read data available in the interface in input[1].
    while(busConnector->readAvailableUart(nullptr, 0)){
        //Read the first character in the buffer from the interface in input[1].
        output[readCount] = busConnector->readUart(nullptr, 0);
        readCount++;
        //If the requested length is reached, stop the loop.
        if(readCount > 100){
            break;
        }
    }
    output[readCount] = '\0';
    Tools::log(INFO,"TC","CC","Received from Calypso: %s",output);
    free(output);
}


void CalypsoCommunication::sendSingleMessage(Component &comp, char* message,int length){
    busConnector->sendUart(&comp,0,921600,message,length);
}

CalypsoCommunication::CalypsoCommunication(BusConnector &busCon){
    busConnector = &busCon;
}
CalypsoCommunication::CalypsoCommunication(){}  