 #include "receiver.h"
#include <QtSql/QSqlDatabase>
#include <QtSql/QtSql>
#include <string>
#include <ctime>
#include <cstdlib>
#include <deque>
#include <chrono>

#define NUM_TIME_BYTES 4
#define NUM_DATE_BYTES 2
#define NUM_ID_BYTES 4
#define NUM_DATA_BYTES 8
#define MESSAGE_LENGTH (NUM_TIME_BYTES + NUM_DATE_BYTES + NUM_ID_BYTES + NUM_DATA_BYTES)

#define NUM_THREADS_NON_LIVE 100
#define NUM_THREADS_LIVE 100


bool storeMessages(char* message,int numMessages,int threadID,  QCanFrameProcessor &frameProcessor, std::map<int,QString> &messageNames, QString &dbIP, int &dbPort, QString &dbUser, QString &dbPassword, QString &datasetName, bool &isLive);
bool formatTableCreationString(QString &result, QString tableName, QVariantMap map);
bool formatInsertString(QString &result, QString tableName, QString time, QVariantMap map);
bool formatFullLogInsertString(QString &result, QString messageName, QString time, QVariantMap map);
bool decodeCanMessage(char* message, QCanFrameProcessor &frameProcessor, QString &messageName, QVariantMap &values, QString &time, std::map<int,QString> &messageNames, bool &isLive);

Receiver::Receiver() {
    WSAData winSocketData;
    int err = WSAStartup(MAKEWORD(2,2),&winSocketData); //sets version of winsocket
    if(err != 0){
        std::cout << "WSASTrutup failed!" << std::endl;
    }


}

bool Receiver::loadDBC(QString path){
    //first reset, so it also works if we load a new file
    frameProcessor.clearMessageDescriptions();
    messageNames.clear();

    QCanDbcFileParser parser;

    if(!parser.parse(path)){
        qInfo() << "Loading DBC failed, with: " << parser.errorString();
        return false;
    }
    qInfo() << "Warnings: " << parser.warnings();

    QList<QCanMessageDescription> messageDescriptions =  parser.messageDescriptions();

    QCanUniqueIdDescription idDes;

    idDes.setBitLength(29);
    frameProcessor.setUniqueIdDescription(idDes);


    qInfo() << "Loaded follwing messges from the DBC:";
    for( int i = 0; i< messageDescriptions.length(); i++){
        messageDescriptions[i].setSize(8); //set this so all messages expect 8 bytes (otherwise the frame Processor will fail on some messages)
        QString name = messageDescriptions[i].name();
        QtCanBus::UniqueId id = messageDescriptions[i].uniqueId();
        messageNames[(int)id] = name;

        qInfo() << "   Name: " << name << ", ID: " << id;

    }
    frameProcessor.addMessageDescriptions(messageDescriptions);
    return true;
}


bool Receiver::setServerAdress(QString IP, int port){
    if(!IP.contains(QRegularExpression("(\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}$)"))){
        return false;
    }
    if (port < 0){
        return false;
    }

    serverAdress.sin_family = AF_INET;
    serverAdress.sin_port = htons(port);
    serverAdress.sin_addr.s_addr = inet_addr(IP.toStdString().c_str());
    return true;
}


bool Receiver::setLocalDBParameters(QString IP, int port, QString user, QString password, QString datasetName){
    if(!IP.contains(QRegularExpression("(\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}$)"))){
        return false;
    }
    if (port < 0){
        return false;
    }
    dbIP = IP;
    dbPort = port;
    dbUser = user;
    dbPassword = password;
    this->datasetName = datasetName;
    return true;
}


bool Receiver::receive() {
    serverSocket = socket(AF_INET,SOCK_DGRAM,0);
    if (serverSocket == INVALID_SOCKET) {
        return false;
    }

    int err = bind(serverSocket, (SOCKADDR *)&serverAdress,sizeof(serverAdress));
    if(err != 0){
        err = WSAGetLastError();
        return false;
    }
    running = true;
    mainThread = std::thread([=] {this->loop(running,serverSocket);});
    for(int i = 0; i < NUM_THREADS_LIVE; i++){
        messageWorkerThreadList.push_back(std::thread([=] {this->messageWorkerThreadLive(running,i);}));
    }

    //set time values for this session
    startTimeMillis = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 1000;
    time_t t = std::time(0);
    startTimeRest = std::gmtime(&t);
    return true;
}

void Receiver::close(){
    running = false;
    closesocket(serverSocket); //close socket to ensure quick end of thread
    std::string port = std::to_string((int)serverAdress.sin_port);
    setsockopt(serverSocket,SOL_SOCKET,SO_REUSEADDR,port.c_str(),sizeof(port.c_str())); //makes the port reusable immediately
    mainThread.join();

    for(int i = 0; i < NUM_THREADS_LIVE; i++){
        messageWorkerThreadList.back().join();
        messageWorkerThreadList.pop_back();
    }
}


bool Receiver::loop(bool &running,SOCKET serverSocket){
    int bytesReceived;
    int serverBufLen = 10000;
    char serverBuf[serverBufLen];

    sockaddr_in SenderAddr;
    int SenderAddrSize = sizeof (SenderAddr);
    int counter = 0;

    while(running){
        bytesReceived = recvfrom(serverSocket, serverBuf, serverBufLen, 0 /* no flags*/, (SOCKADDR *) & SenderAddr, &SenderAddrSize);
        if(bytesReceived == -1){
            qDebug() << "Error while receiving bytes. (If thrown after stopping this is fine.)";
            continue;
        }
        counter++;


        struct message mes = {""};
        int numStuffs = 0;
        for(int i = 0; i < bytesReceived;i++){
            memcpy((mes.data + i),(serverBuf + i + numStuffs),1);
            if(serverBuf[i + numStuffs] == (char) 0xFF){
                numStuffs++;
            }
        }

        messageListMutex.lock();
        messageList.push_back(mes);
        messageListMutex.unlock();


        numTotalMessages += 1;
    }
    return true;
}

void Receiver::messageWorkerThreadLive(bool &running, int threadID){
    struct message mes = {""};
    while(running){
        messageListMutex.lock();
        if(!messageList.empty()){
            memcpy(mes.data,messageList.back().data,18);
            messageList.pop_back();
            messageListMutex.unlock();

            bool t_val = true; //for some reason cant enter false directly
            storeMessages(mes.data,1,threadID,frameProcessor, messageNames, dbIP, dbPort, dbUser, dbPassword, datasetName, t_val);
        }
        messageListMutex.unlock();

    }
}

void Receiver::messageWorkerThreadSD(bool &running, int threadID){
    struct message mes = {""};
    while(running || !messageList.empty()){
        messageListMutex.lock();
        if(!messageList.empty()){
            memcpy(mes.data,messageList.back().data,18);
            messageList.pop_back();

            messageListMutex.unlock();
            bool t_val = false; //for some reason cant enter false directly
            storeMessages(mes.data,1,threadID,frameProcessor, messageNames, dbIP, dbPort, dbUser, dbPassword, datasetName, t_val);
        }
        messageListMutex.unlock();

    }
}


void Receiver::storeMessagesBulk(char *allMessages, int byteCount){
    runningBulkThreads = true;
    for(int i = 0; i < NUM_THREADS_LIVE; i++){
        messageWorkerThreadList.push_back(std::thread([=] {this->messageWorkerThreadSD(runningBulkThreads,i);}));
    }

    struct message mes = {""};
    int posInMessage = 0;
    for(int i = 0; i < byteCount;){
        if(allMessages[i] != (char)0xFF){
            //normal byte
            memcpy((mes.data + posInMessage),(allMessages + i),1);
            posInMessage++;
        }
        else if(allMessages[i] == (char)0xFF && i + 1 < byteCount && allMessages[i + 1] == (char)0xFF){
            //finish this message
            messageListMutex.lock();
            messageList.push_back(mes);
            messageListMutex.unlock();
            mes = {""};//Reset
            posInMessage = 0;
            i++;
        }else{
            //next byte is stuff byte
            memcpy((mes.data + posInMessage),(allMessages + i),1);
            posInMessage++;
            i++;
        }
        i++;
    }

    runningBulkThreads = false;
    //they should keep running until the list is empty
    for(int i = 0; i < NUM_THREADS_LIVE; i++){
        messageWorkerThreadList.back().join();
        messageWorkerThreadList.pop_back();
    }
}

bool Receiver::storeMessagesPublic(char* message,int numMessages){

    std::deque<std::thread*> threadVec;
    char* serverBufReadLocaction = message;
    int numRemainingMessages = numMessages;
    for(int i = 0; i < NUM_THREADS_NON_LIVE; i++){
        int messagesForThisThread = std::min(((int) std::ceil(((double)numMessages) / NUM_THREADS_NON_LIVE)), numRemainingMessages);

        bool f_val = false; //for some reason cant enter false directly
        threadVec.push_back(new std::thread(storeMessages,serverBufReadLocaction,messagesForThisThread,i,std::ref(frameProcessor),std::ref(messageNames),std::ref(dbIP),std::ref(dbPort),std::ref(dbUser),std::ref(dbPassword),std::ref(datasetName),std::ref(f_val)));

        serverBufReadLocaction += messagesForThisThread * MESSAGE_LENGTH;
        numRemainingMessages -= messagesForThisThread;
        if(numRemainingMessages == 0){
            break;
        }
    }
    while(!threadVec.empty()){
        std::thread* thr = threadVec.front();
        threadVec.pop_front();
        thr->join();
        delete thr;
    }
    return true;
}

int Receiver::getNumTotalMessages(){
    return numTotalMessages;
}

long long int Receiver::getStartTime(){
    return startTime;
}

void Receiver::resetMeasurement(){
    numTotalMessages = 0;
    startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

int Receiver::getNumMessagesInQueue(){
    return messageList.size();
}


bool storeMessages(char* message,int numMessages, int threadID, QCanFrameProcessor &frameProcessor,  std::map<int,QString> &messageNames, QString &dbIP, int &dbPort, QString &dbUser, QString &dbPassword, QString &datasetName, bool &isLive){
    //qDebug() << "ThreadID: " << threadID << ", #mes: " << numMessages;
    {
        QSqlDatabase db = QSqlDatabase::addDatabase("QMYSQL",QString::number(threadID));
        db.setHostName(dbIP);
        db.setPort(dbPort);
        db.setDatabaseName(datasetName);
        db.setUserName(dbUser);
        db.setPassword(dbPassword);



        if(!db.open()){
            QSqlError err = db.lastError();
            qDebug() << "DB connection failed, with: " << err;
            return 0;
        }

        QStringList existingTables = db.tables();
        char* messagePointer = message;

        QSqlQuery query(db);
        QString messageName;
        QVariantMap values;
        QString time;
        for (int i = 0; i< numMessages; i++){
            messagePointer = message + i *  MESSAGE_LENGTH;

            //canConversion
            if(!decodeCanMessage(messagePointer,frameProcessor,messageName,values,time,messageNames,isLive)){
                qDebug() << "Translation failed.";
                continue;
            }
            //qDebug() << "Message:" << messageName << ",processed by Thread:" << threadID;


            //write to table
            QString insertQuery;
            formatInsertString(insertQuery,messageName,time,values);
            if(!query.exec(insertQuery)){
                qDebug() << "   Failed Insert: (" << insertQuery << " ), with error: " << query.lastError() << "|| Trying to create table.";
                if(!existingTables.contains(messageName)){
                    //create table
                    QString tableCreationQuery;
                    formatTableCreationString(tableCreationQuery, messageName, values);
                    if(!query.exec(tableCreationQuery)){
                        qDebug() << "Failed Table Creation: (" << tableCreationQuery << " ), with error: " << query.lastError() << "|| This might be due to parallel execution. Therefore an insert will still be tried.";
                        continue;
                    }
                    existingTables.append(messageName);
                    query.exec(insertQuery);
                }
            }

            //write to full log table (maybe add as setting)
            /*formatFullLogInsertString(insertQuery,messageName,time,values);
        if(!query.exec(insertQuery)){
            qDebug() << "Failed Insert: (" << insertQuery << " ), with error: " << query.lastError();
            if(!existingTables.contains("FullLog")){
                //create Full Log table
                QString tableCreationQuerry = "CREATE TABLE FullLog (time TIMESTAMP(3) DEFAULT CURRENT_TIMESTAMP(3), Message VARCHAR(255))";
                if(!query.exec(tableCreationQuerry)){
                    qDebug() << "Failed Log Table Creation: (" << tableCreationQuerry << " ), with error: " << query.lastError() << "|| This might be due to parallel execution. Therefore an insert will still be tried.";
                }
                existingTables.append("FullLog");
            }
            query.exec(insertQuery);
        }
        */

        }
        db.close();
    } //scope added, so that removeDatabase wouldnt throw an error. It needs the db to be outof scope
    QSqlDatabase::removeDatabase(QString::number(threadID));
    return 1;
}

bool decodeCanMessage(char* message, QCanFrameProcessor &frameProcessor, QString &messageName, QVariantMap &values, QString &time, std::map<int,QString> &messageNames, bool &isLive){
    char *dataBytesStart = message;
    char *idBytesStart = (message + NUM_DATA_BYTES);
    char *timeBytesStart = (message + NUM_DATA_BYTES + NUM_ID_BYTES);
    char *dateBytesStart = (message + NUM_DATA_BYTES + NUM_ID_BYTES + NUM_TIME_BYTES);

    unsigned int id = 0;
    memcpy(&id,idBytesStart,NUM_ID_BYTES);

    QCanBusFrame frame;
    frame.setFrameId(id);
    frame.setPayload(QByteArray::fromRawData(dataBytesStart,NUM_DATA_BYTES));

    QCanFrameProcessor::ParseResult res = frameProcessor.parseFrame(frame);
    if(res.signalValues.isEmpty()){
        qDebug() << "Parsing frame with DBC failed, with Error: " << frameProcessor.errorString();
        return false;
    }
    values = res.signalValues;
    messageName = messageNames[id];

    unsigned int timeInt = 0; //time of day in milliseconds if using GPS Time or time in milliseconds since last restart of the telematics board
    memcpy(&timeInt,timeBytesStart,NUM_TIME_BYTES);

    if(Receiver::firstReceivedMillis == -1 || timeInt < Receiver::firstReceivedMillis){
        Receiver::firstReceivedMillis = timeInt;

        Receiver::startTimeMillis = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 1000;
        time_t t = std::time(0);
        Receiver::startTimeRest = std::gmtime(&t);
    }

    unsigned short dateInt = 0;
    memcpy(&dateInt,dateBytesStart,NUM_DATE_BYTES);

    if (dateInt == 0 && !isLive){
        //use server time, for prerecorded data
        time_t t = std::time(0);
        struct tm * now = std::gmtime(&t);


        std::chrono::milliseconds millis(timeInt);
        int hours = std::chrono::duration_cast<std::chrono::hours>(millis).count();
        int minutes = std::chrono::duration_cast<std::chrono::minutes>(millis).count() % 60;
        int seconds = std::chrono::duration_cast<std::chrono::seconds>(millis).count() % 60;
        int milliSeconds = millis.count() % 1000;

        char buffer[sizeof("9999-12-31 29:59:59.999")];
        sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d.%d",
                now->tm_year + 1900,
                now->tm_mon + 1,
                now->tm_mday,
                hours,
                minutes,
                seconds,
                milliSeconds);
        time = QString(buffer);

        return true;
    }
    if(dateInt == 0 && isLive){
        timeInt -= Receiver::firstReceivedMillis;

        //use server time for live data (Start time of receiver + time since telematics start)
        std::chrono::milliseconds millis(timeInt);
        int hours = std::chrono::duration_cast<std::chrono::hours>(millis).count();
        int minutes = std::chrono::duration_cast<std::chrono::minutes>(millis).count() % 60;
        int seconds = std::chrono::duration_cast<std::chrono::seconds>(millis).count() % 60;
        int milliSeconds = millis.count() % 1000;

        if(Receiver::startTimeMillis + milliSeconds >999){
            seconds++;
            milliSeconds -= 1000;
        }
        if(Receiver::startTimeRest->tm_sec + seconds >59){
            minutes++;
            seconds -= 60;
        }
        if(Receiver::startTimeRest->tm_min + minutes >59){
            hours++;
            minutes -= 60;
        }
        int day = 0;
        if(Receiver::startTimeRest->tm_hour + hours >23){
            day = 1;
            hours -= 24;
        }

        char buffer[sizeof("9999-12-31 29:59:59.999")];
        sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d.%d",
                Receiver::startTimeRest->tm_year + 1900,
                Receiver::startTimeRest->tm_mon + 1,
                Receiver::startTimeRest->tm_mday + day,
                Receiver::startTimeRest->tm_hour + hours,
                Receiver::startTimeRest->tm_min + minutes,
                Receiver::startTimeRest->tm_sec + seconds,
                Receiver::startTimeMillis + milliSeconds);
        time = QString(buffer);
        return true;
    }

    //use hardware time
    std::chrono::milliseconds millis(timeInt);
    int hours = std::chrono::duration_cast<std::chrono::hours>(millis).count();
    int minutes = std::chrono::duration_cast<std::chrono::minutes>(millis).count() % 60;
    int seconds = std::chrono::duration_cast<std::chrono::seconds>(millis).count() % 60;
    int milliSeconds = millis.count() % 1000;

    int years = std::floor(dateInt / 366);
    dateInt = dateInt % 366;
    int months = std::floor(dateInt / 31);
    dateInt = dateInt % 31;
    int days = dateInt;

    time = QString::number(years) + "-" + QString::number(months) + "-" + QString::number(days) + " " + QString::number(hours) + ":" + QString::number(minutes) + ":" + QString::number(seconds) + "." + QString::number(milliSeconds);

    return true;
}

bool formatTableCreationString(QString &result, QString tableName, QVariantMap map){
    result = "";
    result.append("CREATE TABLE " + tableName + " (time TIMESTAMP(3) DEFAULT CURRENT_TIMESTAMP(3)");

    for (auto [key, value] : map.asKeyValueRange()) {
        QString dataType = Receiver::convertCppTypeToSQLType(value);
        if(dataType == ""){
            return 0;
        }
        result.append("," + key + " " + dataType);
    }
    result.append(")");
    return 1;
}

bool formatInsertString(QString &result, QString tableName, QString time, QVariantMap map){
    result = "";
    result.append("INSERT INTO " + tableName + " VALUES ('" + time + "'");
    for (auto [key, value] : map.asKeyValueRange()) {
        if(((QString) value.typeName()).compare("QString") == 0){
            result.append(",'" + value.toString() + "'");
        }else{
            result.append("," + value.toString());
        }
    }
    result.append(")");
    return 1;
}

bool formatFullLogInsertString(QString &result, QString messageName, QString time, QVariantMap map){
    result = "";
    result.append("INSERT INTO FullLog VALUES ('" + time + "', '" + messageName + ": {");
    for (auto [key, value] : map.asKeyValueRange()) {
        result.append(",`" + key + "`: " + value.toString());

    }
    result.append("}')");
    return 1;
}

QString Receiver::convertCppTypeToSQLType(QVariant value){
    QString str = value.typeName();
    if(str.compare("int") == 0){
        return "INT";
    }
    if(str.compare("double") == 0){
        return "DOUBLE";
    }
    if(str.compare("float") == 0){
        return "FLOAT";
    }
    if(str.compare("QString") == 0){
        return "VARCHAR(255)";
    }
    if(str.compare("string") == 0){
        return "VARCHAR(255)";
    }
    if(str.compare("bool") == 0){
        return "BOOL";
    }
    qDebug() << "Conversion failed. Did not know type: " << str;
    return "";
}

