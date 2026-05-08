#include "GPSCommunication.h"
#include <sstream>
#include <locale>


GPSCommunication::GPSCommunication(BusConnector &busCon, CanBus *canBus){
    busConnector = &busCon;
    this->canBus = canBus;
}

void GPSCommunication::setup(Component &comp){
    Tools::log(INFO, "TC","GPSCom","---------------------");

    Tools::log(INFO, "TC","GPSCom","Enable: %d",busConnector->enableUart(&comp,1));
    Tools::log(INFO, "TC","GPSCom","receive: %d",busConnector->startReceivingUart(&comp,1,9600));
    
    sendCommand(comp,0);
    sendCommand(comp,1);
    
}

void GPSCommunication::sendCommand(Component &comp,u_int commandID){
    char* commands[2] = {"$PMTK220,100*2F","$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"};
    Tools::log(INFO, "TC","GPSCom","Send Command: %s",commands[commandID]);
    busConnector->sendUart(&comp,1,9600,commands[commandID]);
}



void GPSCommunication::readData(Component &comp, bool &newTime, uint32_t &time, uint16_t &date) {

    if(busConnector->readAvailableUart(nullptr, 1)){
        u_int8_t numComma = 0;
        char data[11][15];
        uint8_t readCount = 0;
        bool isValid = false;
        //Read as long as there is uart read data available in the interface in input[1].
        while(busConnector->readAvailableUart(nullptr, 1)){
            char letter = busConnector->readUart(nullptr, 1);
            if(letter == ',' || letter == '\n'){
                data[numComma][readCount] = '\0';
                numComma++;
                readCount = 0;
                if(numComma - 1 == 0 && 0 != strcmp(data[numComma -1],"$GPRMC")){
                    numComma = 0;
                }
                if(numComma == 11){
                    isValid = true;
                    break; //end of message
                }
            }else{
                data[numComma][readCount] = letter;
                readCount++;
                //If the requested length is reached, stop the loop.
                if(readCount > 14){
                    Tools::log(INFO,"TC","GPSCom","This state should not be reached");
                    break;
                }
            }   
        }

        while(busConnector->readAvailableUart(nullptr, 1)){
            busConnector->readUart(nullptr,1);
        }
        if(isValid){
            Tools::log(INFO,"TC","GPSCom","GPS: Valid %s (A - Good, V - Bad), Time %s, Lat: %s, Long: %s",data[2],data[1],data[3],data[5]);
            //All The data included in the message being logged:
            /*char* info[12] = {"Tag","Time","Status","Latitude","Laitude N/S","Longitude","Longitude E/W","Speed over Ground","Course over Ground","Date", "Magentic Variation","Mode and Checksum"};
            for(int i= 0; i< 12 ; i++){
                Tools::log(INFO,"TC","GPSCom","GPS - %s: %s",info[i],data[i]);
            }*/


            //decode time
            uint8_t hours = (data[1][0] - '0') * 10 +  (data[1][1] - '0');
            uint8_t minutes = (data[1][2] - '0') * 10 +  (data[1][3] - '0');
            uint8_t seconds = (data[1][4] - '0') * 10 +  (data[1][5] - '0');
            uint16_t milliSeconds = (data[1][7] - '0') * 100 +  (data[1][8] - '0') * 10 +  (data[1][9] - '0'); //start at 7 because of the seperating "."

            uint8_t day = (data[9][0] - '0') * 10 +  (data[9][1] - '0');
            uint8_t month = (data[9][2] - '0') * 10 +  (data[9][3] - '0');
            uint8_t year = (data[9][4] - '0') * 10 +  (data[9][5] - '0');

            if(year != 80){ //year 80 means the modules hasn't received a date yet (5.1.80)
                newTime = true;
                time = milliSeconds + seconds * 1000 + minutes * 60000 + hours * 3600000;
                date = day + month * 31 + year * 366;
            }

            //send coordinates
            std::istringstream strLat(data[3]);
            strLat.imbue(std::locale("C"));
            int lat;
            strLat >> lat;
            lat *= 100;

            std::istringstream strLon(data[3]);
            strLon.imbue(std::locale("C"));
            int lon;
            strLon >> lon;
            lon *= 100;

            //if(lat != 0 && lon != 0){
                CanMessage *coordsMessage = new CanMessage(122);

                memcpy(coordsMessage->getData(),&lat,4);
                memcpy(coordsMessage->getData() + 4,&lat,4);
                
                canBus->send(coordsMessage); //send on can bus
                canBus->addUnhandledCanMessage(coordsMessage); //send via wifi
            //}
            
        }
    }
}