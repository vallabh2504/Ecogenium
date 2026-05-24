#pragma once
#include "src/Atava/Tasks/Task.h"
#include "src/Atava/Utils/Tools.h"
#include "src/Atava/Hardware/BusConnector.h"
#include "src/Atava/Defines.h"

#include "src/include/arduino/Arduino.h"


#include "libraries/CalypsoCommunication.h"
#include "libraries/GPSCommunication.h"
#include "libraries/SDCommunication.h"

//This can be set if a GPS connection can't be established. Then the logging will use server time instead of the hardware time.
#define USE_SERVER_TIME
 
union int32_bytes{
    uint32_t i;
    unsigned char bytes[sizeof(uint32_t)];
};

union int16_bytes{
    uint16_t i;
    unsigned char bytes[sizeof(uint16_t)];
};

class TelematicsComponent: public Component {
    private:
    
        uint32_t time;
        uint16_t date;
        MillisTimer timer = MillisTimer();

        uint32_t numCanMessges;

        int messageAddByteWithByteStuffing(char* message,unsigned char byte,bool firstByteOfMessage,bool doStuffing){
            //message needs to have allocated enough space (20 Byte + 18 Byte)
            static int i;
            if(firstByteOfMessage){
                i = 0;
            }else{
                i++;
            }

            message[i] = byte;
            if(byte == 0xFF && doStuffing){
                i++;
                message[i] = 0x00;
            }
            return i + 1; //return length of message in Bytes
        }

        int formatMessage(uint32_t id, byte_t* bytes, uint32_t time, uint16_t date, char* message){
            memset(message, 0, 38 * sizeof(char)); 
            //requires message to have enough allocated memory
            union int32_bytes id_bytes;
            id_bytes.i = id;

            union int32_bytes time_bytes;
            time_bytes.i = time;

            union int16_bytes date_bytes;
            date_bytes.i = date;

            for(int i = 0; i < 8; i++) {
                if(i == 0){
                    messageAddByteWithByteStuffing(message,(char) bytes[i],true,true);
                }else{
                    messageAddByteWithByteStuffing(message,(char) bytes[i],false,true);
                }
            }
            for(int i = 0; i < 4; i++){
                messageAddByteWithByteStuffing(message, id_bytes.bytes[i],false,true);
            }
            for(int i = 0; i < 4; i++){
                messageAddByteWithByteStuffing(message, time_bytes.bytes[i],false,true);
            }
            for(int i = 0; i < 2; i++){
                messageAddByteWithByteStuffing(message, date_bytes.bytes[i],false,true);
            }
            messageAddByteWithByteStuffing(message, 0xFF,false,false);//transparent_trigger_1
            int length = messageAddByteWithByteStuffing(message, 0xFF,false,false);//transparent_trigger_2
            return length;
        }

        char *completeMessage = (char*)malloc(38 * sizeof(char));

    public:
        BusConnector *busConnector;
        CalypsoCommunication *calypsoComm;
        GPSCommunication *gpsComm;
        SDCommunication *sdComm;
        bool newTime = false;


        void _setup(){
            registerLoop(1);//main
            registerLoop(100);//gps

            //CAN setup
            getCanBus()->setAcceptAllCanIDs(true);

            //Uart setup
            busConnector = getBusConnector();

            calypsoComm = new CalypsoCommunication(*busConnector);
            calypsoComm->setup(*this);
            gpsComm = new GPSCommunication(*busConnector,getCanBus());
            gpsComm->setup(*this);
            sdComm = new SDCommunication();
            sdComm->setup();

            #ifdef USE_SERVER_TIME
                timer.start();
                timer.reset();
            #endif
            delay(1000);
        }

        void _disassemble(){
            free(completeMessage);
        }

        void _emergencyTriggered(){

        }

        void _emergencyReset(){

        }

        void _loop(uint8_t loopid){

            if(loopid == 0){
                getCanBus()->loop(); 

                #ifndef USE_SERVER_TIME
                if(newTime){
                    timer.start();
                    timer.reset();
                }
                #endif

                //read Calypso UART
                //calypsoComm->readData(*this);
                
                //read CAN -> write SD and Calypso
                if(getCanBus()->hasUnhandledCanMessages() && hasTime()){//without having a time there is no need to log, as data won't make a lot of sense when reading
                    sdComm->open();
                    while(getCanBus()->hasUnhandledCanMessages()){
                        CanMessage message = getCanBus()->getNextUnhandledCanMessage();
                        
                        numCanMessges++;
                        int length = formatMessage(message.getID(), message.getData(),getCurrentTime(),getCurrentDate(),completeMessage);
                        //Tools::log(INFO,"TC","Main","Received CAN-Messages %d. It was message number: %d",message.getID(),numCanMessges);
                        
                        //Send Calypso
                        calypsoComm->sendSingleMessage(*this, completeMessage,length);

                        //store SD
                        sdComm->writeCanMessage(completeMessage,length);
                    }
                    sdComm->close();
                }
            }else if(loopid == 1){
                newTime = false;
                gpsComm->readData(*this,newTime,time, date); 
            }
            
        }
       
        /**
         * check if a time is  established. This is dependend on a GPS connection or if USE_SERVER_TIME is set it will always be true.
         * @return true if a time is established
         */
        bool hasTime(){
            #ifdef USE_SERVER_TIME
            return true;
            #else
            return timer.isRunning();
            #endif
        }

        /**
         * returns the current time of this chip. If USE_SERVER_TIME is set this will always be zero, otherwise it is the time received from the gps-module + all ticks that happend since then
         * @warning This may lead to problems if used exactly at midnight
         * @return current time
         */
        uint32_t getCurrentTime(){
            #ifdef USE_SERVER_TIME
            return timer.read() * 10; // this is adjusted on the serverside to be relative to the current server time
            #else
            return (time + timer.read() * 10); //ticks are at 100 Hz and timer calcs ticks not time-> times 10 to get the number of milliseconds.
            #endif
        }
        
        /**
         * returns the current date of this chip. If USE_SERVER_TIME is set this will always be zero, otherwise it is the date received from the gps-module
         * @warning This may lead to problems if used exactly at midnight
         * @return current date
         */
        uint16_t getCurrentDate(){
            #ifdef USE_SERVER_TIME
            return 0; //IF using Server time we need to detect a restart. For this reason the unused date is hijacked to transmitt a random value that can be checked for a change
            #else
            return date;            
            #endif
        }

};

Task* factoryTelematicsComponent(){
    return new TelematicsComponent();
}