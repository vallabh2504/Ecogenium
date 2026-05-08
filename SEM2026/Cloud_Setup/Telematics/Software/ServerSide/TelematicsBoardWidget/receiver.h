#ifndef RECEIVER_H
#define RECEIVER_H


#include <winsock2.h>
#include <WS2tcpip.h>
#include <iostream>
#include <QString>
#include <QtSerialBus/QtSerialBus>
#include <ctime>



class Receiver
{
public:
    Receiver();
    bool receive();
    void close();
    bool loadDBC(QString path);
    bool setServerAdress(QString IP, int port);
    bool setLocalDBParameters(QString IP, int port, QString user, QString password, QString datasetName);
    std::map<int,QString> messageNames;
    bool storeMessagesPublic(char* message,int numMessages);
    int getNumTotalMessages();
    long long int getStartTime();
    void resetMeasurement();
    static QString convertCppTypeToSQLType(QVariant value);
    int getNumMessagesInQueue();
    void storeMessagesBulk(char *allMessages, int byteCount);
    inline static int startTimeMillis;
    inline static struct tm * startTimeRest;
    inline static int firstReceivedMillis = -1;



private:
    SOCKET serverSocket = INVALID_SOCKET;
    bool running = false;
    bool runningBulkThreads = false;

    QCanFrameProcessor frameProcessor;
    bool loop(bool &running,SOCKET serverSocket);

    int numTotalMessages = 0;
    long long int startTime;

    std::thread mainThread;

    struct message {char data[18];};
    std::vector<struct message> messageList;
    std::mutex messageListMutex;
    std::vector<std::thread> messageWorkerThreadList;

    void messageWorkerThreadLive(bool &running, int threadID);
    void messageWorkerThreadSD(bool &running, int threadID);



    struct sockaddr_in serverAdress;
    QString dbIP;
    int dbPort;
    QString dbUser;
    QString dbPassword;
    QString datasetName;




};

#endif // RECEIVER_H
