  #include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QtSql/QSqlDatabase>
#include <QtSql/QtSql>
#include <QTableView>
#include <QFile>
#include <windows.h>
#include <shellapi.h>
#include <wlanapi.h>
#include <QTimer>
#include <set>
#include <QFile>
#include <deque>



MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->btnStartReceive,SIGNAL(clicked()),this,SLOT(start()));
    connect(ui->btnStopReceive,SIGNAL(clicked()),this,SLOT(stop()));
    connect(ui->btnOpenGrafana,SIGNAL(clicked()),this,SLOT(openGrafana()));
    connect(ui->btnLoadDBC, SIGNAL(clicked()),this,SLOT(loadDBC()));
    connect(ui->btnLoadFile, SIGNAL(clicked()),this,SLOT(readMessageFile()));
    connect(ui->btnRunLocalSQLQuery, SIGNAL(clicked()),this,SLOT(runLocalSQLQuery()));
    connect(ui->btnDeleteLocalDB, SIGNAL(clicked()),this,SLOT(deleteLocalDB()));
    connect(ui->btnRefresh, SIGNAL(clicked()),this,SLOT(refreshStatus()));
    connect(ui->btnStartMeasurement, SIGNAL(clicked()),this,SLOT(startMeasurement()));
    connect(ui->btnUploadMessages,SIGNAL(clicked()),SLOT(uploadData()));
    connect(ui->btnDownloadMessages,SIGNAL(clicked()),SLOT(downloadData()));
    connect(ui->btnDatasetsNameRefresh,SIGNAL(clicked()),SLOT(refreshSettingsDatasetNames()));
    connect(ui->btnSyncDatasetsNameRefreshServer,SIGNAL(clicked()),SLOT(refreshSyncDatasetNamesServer()));
    connect(ui->btnSyncDatasetsNameRefreshLocal,SIGNAL(clicked()),SLOT(refreshSyncDatasetNamesLocal()));
    connect(ui->inDatasetName,SIGNAL(activated(int)),SLOT(settingsDatsetNameActicatedLocal(int)));
    connect(ui->inSyncDatasetNameLocal,SIGNAL(activated(int)),SLOT(syncDatsetNameActicatedLocal(int)));
    connect(ui->inSyncDatasetNameServer,SIGNAL(activated(int)),SLOT(syncDatsetNameActicatedServer(int)));
    connect(ui->btnExportAsCSV,SIGNAL(clicked()),SLOT(exportDatasetAsCSV()));

    uiStat = ui;
    //set output to application and or file
#if defined(LOGTOFILE) | defined(LOGTOCONSOLE) | defined(LOGTOWIDGET)
    qInstallMessageHandler(MainWindow::customMessageOutput);
#endif

    ui->uploadProgressbar->hide();

    //load DBC from predefined path for ease of use TODO: make it system independent
    reci.loadDBC(".\\messages.dbc");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::start(){

    if(!reci.setServerAdress(ui->inLocalIP->text(),ui->inLocalPort->value())){
        QMessageBox::information(this,"Info","Either the local IP or Port have a wrong format.");
        return;
    }
    if(!reci.setLocalDBParameters(ui->inLocalDBIP->text(),ui->inLocalDBPort->value(), ui->inLocalDBUser->text(), ui->inLocalDBPassword->text(),ui->inDatasetName->currentText())){
        QMessageBox::information(this,"Info","Either the local DB IP or Port have a wrong format.");
        return;
    }
    if(!reci.receive()){
        QMessageBox::information(this,"Info","The socket couldn't be created!");
        return;
    }
    qInfo() << "Start Receiving!";
    ui->btnStopReceive->setEnabled(true);
    ui->btnStartReceive->setEnabled(false);
    ui->btnDeleteLocalDB->setEnabled(false);
    ui->btnLoadFile->setEnabled(false);
    ui->btnLoadDBC->setEnabled(false);
    ui->boxSettings->setEnabled(false);
    ui->btnExportAsCSV->setEnabled(false);
}

void MainWindow::stop(){
    qInfo() << "Stop Receiving!";

    reci.close();
    ui->btnStopReceive->setEnabled(false);
    ui->btnStartReceive->setEnabled(true);
    ui->btnDeleteLocalDB->setEnabled(true);
    ui->btnLoadFile->setEnabled(true);
    ui->btnLoadDBC->setEnabled(true);
    ui->boxSettings->setEnabled(true);
    ui->btnExportAsCSV->setEnabled(true);
}

void MainWindow::openGrafana(){
    ShellExecute(0, 0, L"http://127.0.0.1:3000", 0, 0 , SW_SHOW );
}

void MainWindow::refreshStatus(){
    ui->btnRefresh->setEnabled(false);
    int numMessagesAtStart = reci.getNumTotalMessages();
    ui->labelDBStatus->setText(QString("Local DB: ..."));
    ui->labelDBServerStatus->setText(QString("Server DB: ..."));
    ui->labelReceivingStatus->setText(QString("Receiving Messages: ..."));
    ui->labelMessagesInQueue->setText(QString("Messages In Queue: ..."));

    QTimer::singleShot(2 * 1000,this,[this,numMessagesAtStart] (){this->refreshStatusDelayed(numMessagesAtStart);});
}

void MainWindow::refreshStatusDelayed(int numMessagesAtStart){
    QSqlDatabase db = QSqlDatabase::addDatabase("QMYSQL","StatusLocal");
    db.setHostName(ui->inLocalDBIP->text());
    db.setPort(ui->inLocalDBPort->value());
    db.setDatabaseName(ui->inDatasetName->currentText());
    db.setUserName(ui->inLocalDBUser->text());
    db.setPassword(ui->inLocalDBPassword->text());

    if(db.open()){
        ui->labelDBStatus->setText(QString("Local DB: <span style=\"color:green;\">") + QChar(0x2714) + QString("</span>"));
        db.close();
    }else{
        ui->labelDBStatus->setText(QString("Local DB: ") + QChar(0x274C));
    }

    QSqlDatabase dbServer = QSqlDatabase::addDatabase("QMYSQL","StatusServer");
    dbServer.setHostName(ui->inServerDBIP->text());
    dbServer.setPort(ui->inServerDBPort->value());
    dbServer.setDatabaseName(ui->inSyncDatasetNameServer->currentText());
    dbServer.setUserName(ui->inServerDBUser->text());
    dbServer.setPassword(ui->inServerDBPassword->text());

    if(dbServer.open()){
        ui->labelDBServerStatus->setText(QString("Server DB: <span style=\"color:green;\">") + QChar(0x2714) + QString("</span>"));
        dbServer.close();
    }else{
        ui->labelDBServerStatus->setText(QString("Server DB: ") + QChar(0x274C));
    }

    if(reci.getNumTotalMessages() > numMessagesAtStart){
        ui->labelReceivingStatus->setText(QString("Receiving Messages: <span style=\"color:green;\">") + QChar(0x2714) + QString("</span>"));

    }else{
        ui->labelReceivingStatus->setText(QString("Receiving Messages: ") + QChar(0x274C));
    }


    ui->labelMessagesInQueue->setText(QString("Messages In Queue:").append(QString::number(reci.getNumMessagesInQueue())));

    ui->btnRefresh->setEnabled(true);
}

void MainWindow::measurementDelayed(){
    long long int time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() -  reci.getStartTime();
    int messageCount = reci.getNumTotalMessages();
    double mps = ((double)messageCount * 1000) / time;
    ui->labelMpS->setText(QString("Messages / Second: ") + QString::number(mps));
    ui->labelMessageCount->setText(QString("#Messages: ") + QString::number(messageCount));
    ui->btnStartMeasurement->setEnabled(true);
}

void MainWindow::startMeasurement(){
    ui->btnStartMeasurement->setEnabled(false);
    reci.resetMeasurement();
    QTimer::singleShot(ui->inMeasureTime->value() * 1000,this,&MainWindow::measurementDelayed);
}

void MainWindow::runLocalSQLQuery(){
    if(ui->inDatasetName->currentText() == ""){
        QMessageBox::information(this,"Info","The dataset name shouldn't be empty!");
        return;
    }

    if(!reci.setLocalDBParameters(ui->inLocalDBIP->text(),ui->inLocalDBPort->value(), ui->inLocalDBUser->text(), ui->inLocalDBPassword->text(),ui->inDatasetName->currentText())){
        QMessageBox::information(this,"Info","Either the local DB IP or Port have a wrong format.");
        return;
    }

    QSqlDatabase db = QSqlDatabase::addDatabase("QMYSQL",QString::number(1001));
    db.setHostName(ui->inLocalDBIP->text());
    db.setPort(ui->inLocalDBPort->value());
    db.setDatabaseName(ui->inDatasetName->currentText());
    db.setUserName(ui->inLocalDBUser->text());
    db.setPassword(ui->inLocalDBPassword->text());
    if(!db.open()){
        QMessageBox::information(this,"Info","Database couldn't be reached!");
        return;
    }

    QString queryString = QInputDialog::getMultiLineText(this,"Local SQL Querry on Dataset: " + ui->inDatasetName->currentText(),"Enter your Querry:","e.g: \nSHOW tables \nSELECT * FROM FullLog");
    if(queryString.isEmpty()){
        db.close();
        return;
    }

    QSqlQueryModel *model = new QSqlQueryModel;
    model->setQuery(queryString,db);
    QTableView *view = new QTableView;
    view->setModel(model);
    view->setWindowTitle("Query Result");
    view->setWindowIcon(QIcon::fromTheme(QIcon::ThemeIcon::NetworkWireless));
    view->show();

    db.close();
    return;
}

void MainWindow::deleteLocalDB(){
    if(ui->inDatasetName->currentText() == ""){
        QMessageBox::information(this,"Info","The dataset name shouldn't be empty!");
        return;
    }

    QMessageBox::StandardButton reply;
    reply = (QMessageBox::StandardButton) QMessageBox::question(this,"Delete Local Dataset","WARNING! \nYou are about to delete all localy stored data in the dataset: '" + ui->inDatasetName->currentText() + "'. \nAre you sure you want to do this?",QMessageBox::Yes,QMessageBox::No);
    if(reply != QMessageBox::Yes){
        return;
    }

    if(!reci.setLocalDBParameters(ui->inLocalDBIP->text(),ui->inLocalDBPort->value(), ui->inLocalDBUser->text(), ui->inLocalDBPassword->text(),ui->inDatasetName->currentText())){
        QMessageBox::information(this,"Info","Either the local DB IP or Port have a wrong format.");
        return;
    }

    QSqlDatabase db = QSqlDatabase::addDatabase("QMYSQL",QString::number(1001));
    db.setHostName(ui->inLocalDBIP->text());
    db.setPort(ui->inLocalDBPort->value());
    db.setDatabaseName(ui->inDatasetName->currentText());
    db.setUserName(ui->inLocalDBUser->text());
    db.setPassword(ui->inLocalDBPassword->text());

    if(!db.open()){
        QMessageBox::information(this,"Info","Database couldn't be reached!");
    }

    QSqlQuery query(db);
    if(query.exec("DROP DATABASE " + ui->inDatasetName->currentText())){
        QMessageBox::information(this,"Info","Dataset dropped!");
        refreshSettingsDatasetNames();
    }
}

void MainWindow::loadDBC(){
    QString fileName = QFileDialog::getOpenFileName(this, tr("Load DBC file"),tr("."), tr("CAN Database Files (*.dbc)"));

    if(reci.loadDBC(fileName)){
        QMessageBox::information(this,"Info","DBC file loaded.");
    }else{
        QMessageBox::information(this,"Info","DBC file couldn't be loaded.");
    }
}

void MainWindow::readMessageFile(){
    qInfo() << "Started reading Messages from File!";

    if(ui->inDatasetName->currentText() == ""){
        QMessageBox::information(this,"Info","The dataset name shouldn't be empty!");
        return;
    }

    QString fileName = QFileDialog::getOpenFileName(this, tr("Load Message file"),tr("."));
    QFile file(fileName);
    if(!file.open(QIODevice::ReadOnly)){
        QMessageBox::information(this,"Info","Couldn't open file.");
        return;
    }

    char *allMessages = (char *) malloc(file.size());
    long long int err = file.read((char*) allMessages,file.size());
    if(err < 0){
        QMessageBox::information(this,"Info","Error while reading file.");
        free(allMessages);
        return;
    }
    file.close();

    if(!reci.setLocalDBParameters(ui->inLocalDBIP->text(),ui->inLocalDBPort->value(), ui->inLocalDBUser->text(), ui->inLocalDBPassword->text(),ui->inDatasetName->currentText())){
        QMessageBox::information(this,"Info","Either the local DB IP or Port have a wrong format.");
        return;
    }

    reci.storeMessagesBulk(allMessages,file.size());

    free(allMessages);
    qInfo() << "Finished reading Messages from File!";
}

void MainWindow::dataSetNameActivatedGeneric(int index, QString password, QString user, QString ip, int port, QComboBox *uiField){
    QString currentText = uiField->currentText();
    if(currentText == ""){
        QMessageBox::information(this,"Info","An empty name isn't allowed for datasets!");
        return;
    }

    if(!ip.contains(QRegularExpression("(\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}$)"))){
        QMessageBox::information(this,"Info","The DB IP has a wrong format.");
        return;
    }
    if (port < 0){
        QMessageBox::information(this,"Info","The DB Port has a wrong format.");
        return;
    }

    QSqlDatabase db = QSqlDatabase::addDatabase("QMYSQL",QString::number(1002));
    db.setHostName(ip);
    db.setPort(port);
    db.setUserName(user);
    db.setPassword(password);

    if(!db.open()){
        QMessageBox::information(this,"Info","Couldn't reach DB!");
        return;
    }

    QSqlQuery query(db);
    if(!query.exec("SHOW DATABASES")){

        QMessageBox::warning(this,"Info","Couldn't querry DB for datasets! Error: " + query.lastError().text());
        return;
    }

    while(query.next()){
        QString dsName = query.value(0).toString();
        if(currentText == dsName){
            return;
        }
    }

    //this should only be reached, iff the dataset name is new -> create new dataset
    QMessageBox::StandardButton reply;
    reply = (QMessageBox::StandardButton) QMessageBox::question(this,"Create new DataSet","Do you want to create a new Dataset called: " + currentText + "?",QMessageBox::Yes,QMessageBox::No);
    if(reply == QMessageBox::No){
        uiField->setCurrentText("");
    }else{
        if(query.exec("CREATE DATABASE " + currentText)){
            QMessageBox::information(this,"Info","New Database created!");
        }
    }
}

void MainWindow::settingsDatsetNameActicatedLocal(int index){
    QString localPassword = ui->inLocalDBPassword->text();
    QString localUser = ui->inLocalDBUser->text();
    QString localIP = ui->inLocalDBIP->text();
    int localPort = ui->inLocalDBPort->value();

    dataSetNameActivatedGeneric(index,localPassword,localUser,localIP,localPort,ui->inDatasetName);
}
void MainWindow::syncDatsetNameActicatedServer(int index){
    QString serverPassword = ui->inServerDBPassword->text();
    QString serverUser = ui->inServerDBUser->text();
    QString serverIP = ui->inServerDBIP->text();
    int serverPort = ui->inLocalDBPort->value();

    dataSetNameActivatedGeneric(index,serverPassword,serverUser,serverIP,serverPort,ui->inSyncDatasetNameServer);
}
void MainWindow::syncDatsetNameActicatedLocal(int index){
    QString localPassword = ui->inLocalDBPassword->text();
    QString localUser = ui->inLocalDBUser->text();
    QString localIP = ui->inLocalDBIP->text();
    int localPort = ui->inLocalDBPort->value();

    dataSetNameActivatedGeneric(index,localPassword,localUser,localIP,localPort,ui->inSyncDatasetNameLocal);

}

void MainWindow::refreshDatasetNameGeneric(QString password, QString user, QString ip, int port, QComboBox *uiField){
    uiField->clear();
    if(!ip.contains(QRegularExpression("(\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}$)"))){
        QMessageBox::information(this,"Info","The DB IP has a wrong format.");
        return;
    }
    if (port < 0){
        QMessageBox::information(this,"Info","The DB Port has a wrong format.");
        return;
    }

    QSqlDatabase dbLocal = QSqlDatabase::addDatabase("QMYSQL",QString::number(1002));
    dbLocal.setHostName(ip);
    dbLocal.setPort(port);
    dbLocal.setUserName(user);
    dbLocal.setPassword(password);

    if(!dbLocal.open()){
        QMessageBox::information(this,"Info","Couldn't reach DB! Did you start docker?");
        return;
    }

    QSqlQuery query(dbLocal);
    if(!query.exec("SHOW DATABASES")){

        QMessageBox::warning(this,"Info","Couldn't query DB for datasets! Error: " + query.lastError().text());
        return;
    }

    std::set<QString> blackList{"information_schema","mysql","performance_schema","sys"};

    while(query.next()){
        QString dsName = query.value(0).toString();
        if(blackList.find(dsName) == blackList.end()){
            uiField->addItem(dsName);
        }
    }
    dbLocal.close();
}

void MainWindow::refreshSettingsDatasetNames(){
    ui->inDatasetName->clear();

    QString localPassword = ui->inLocalDBPassword->text();
    QString localUser = ui->inLocalDBUser->text();
    QString localIP = ui->inLocalDBIP->text();
    int localPort = ui->inLocalDBPort->value();

    refreshDatasetNameGeneric(localPassword,localUser,localIP,localPort,ui->inDatasetName);
}

void MainWindow::refreshSyncDatasetNamesLocal(){
    ui->inSyncDatasetNameLocal->clear();

    QString localPassword = ui->inLocalDBPassword->text();
    QString localUser = ui->inLocalDBUser->text();
    QString localIP = ui->inLocalDBIP->text();
    int localPort = ui->inLocalDBPort->value();

    refreshDatasetNameGeneric(localPassword,localUser,localIP,localPort,ui->inSyncDatasetNameLocal);
}

void MainWindow::refreshSyncDatasetNamesServer(){
    ui->inSyncDatasetNameServer->clear();

    QString localPassword = ui->inServerDBPassword->text();
    QString localUser = ui->inServerDBUser->text();
    QString localIP = ui->inServerDBIP->text();
    int localPort = ui->inServerDBPort->value();

    refreshDatasetNameGeneric(localPassword,localUser,localIP,localPort,ui->inSyncDatasetNameServer);
}


void MainWindow::exportDatasetAsCSV(){
    QString folderName = QFileDialog::getSaveFileName(this,"Where do you want to store the CSVs?",".");


    //DB connection setup
    QString localDataset = ui->inDatasetName->currentText();
    if(localDataset == ""){
        QMessageBox::information(this,"Info","An empty name isn't allowed for datasets!");
        return;
    }

    QString localPassword = ui->inLocalDBPassword->text();
    QString localUser = ui->inLocalDBUser->text();
    QString localIP = ui->inLocalDBIP->text();
    int localPort = ui->inLocalDBPort->value();
    if(!localIP.contains(QRegularExpression("(\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}$)"))){
        QMessageBox::information(this,"Info","The local DB IP has a wrong format.");
        return;
    }
    if (localPort < 0){
        QMessageBox::information(this,"Info","The local DB Port has a wrong format.");
        return;
    }

    QSqlDatabase dbLocal = QSqlDatabase::addDatabase("QMYSQL",QString::number(1002));
    dbLocal.setHostName(localIP);
    dbLocal.setPort(localPort);
    dbLocal.setDatabaseName(localDataset);
    dbLocal.setUserName(localUser);
    dbLocal.setPassword(localPassword);

    if(!dbLocal.open()){
        QMessageBox::information(this,"Info","Couldn't reach local DB!");
        return;
    }

    //create main folder
    QDir directory = QDir(folderName);
    if(directory.exists()){
        QMessageBox::information(this,"Info","Please choose a folder name, that doesn't already exist.");
        return;
    }
    directory.mkdir(folderName);

    //itterate through data and create CSVs for each table
    QSqlQuery tableQuery(dbLocal);
    if(!tableQuery.exec("SHOW tables")){
        QMessageBox::warning(this,"Info","Couldn't query Dataset for tables! Error: " + tableQuery.lastError().text());
        return;
    }
    while(tableQuery.next()){
        QString tableName = tableQuery.value(0).toString();

        //create csv
        QString fileContent ;
        QFile csvFile(folderName + "\\" + tableName + ".csv");
        if (!csvFile.open(QFile::WriteOnly | QFile::Text)){
            qInfo("failed to open csv file");
            continue;
        }

        QSqlQuery messageQuerry(dbLocal);
        if(!messageQuerry.exec("SELECT * FROM " + tableName)){
            QMessageBox::warning(this,"Info","Couldn't query Table (" + tableName + ") for Values! Error: " + messageQuerry.lastError().text());
            continue;
        }

        QTextStream outStream(&csvFile);
        //fill in table collumn names
        QSqlRecord localRecord = messageQuerry.record();
        for (int i = 0; i < localRecord.count(); ++i) {
            QString fieldName = localRecord.fieldName(i);
            if (i>0){
                outStream << ',';
            }
            outStream << fieldName;
        }
        outStream << '\n';

        //fill in data
        while (messageQuerry.next()){
            const QSqlRecord record = messageQuerry.record();
            for (int i=0, recCount = record.count() ; i<recCount ; ++i){
                if (i>0){
                    outStream << ',';
                }
                outStream << escapedCSV(record.value(i).toString());

            }
            outStream << '\n';
        }

    }



    dbLocal.close();
}

QString MainWindow::escapedCSV(QString unexc)
{
    if (!unexc.contains(QLatin1Char(',')))
        return unexc;
    return '\"' + unexc.replace(QLatin1Char('\"'), QStringLiteral("\"\"")) + '\"';
}
void MainWindow::uploadData(){
    ui->btnUploadMessages->setEnabled(false);
    ui->btnDownloadMessages->setEnabled(false);
    ui->boxTools->setEnabled(false);

    upDownloadProgress = 0;
    upDownloadMax = 0;

    new std::thread([this] (){this->upDownloadDataDelayed(true);});
    progressbarTimer = new QTimer(this);
    connect(progressbarTimer, SIGNAL(timeout()), this, SLOT(upDownloadProgressbar()));
    progressbarTimer->start(1000);
    ui->uploadProgressbar->show();
}

void MainWindow::downloadData(){
    ui->btnUploadMessages->setEnabled(false);
    ui->btnDownloadMessages->setEnabled(false);
    ui->boxTools->setEnabled(false);

    upDownloadProgress = 0;
    upDownloadMax = 0;

    new std::thread([this] (){this->upDownloadDataDelayed(false);});
    progressbarTimer = new QTimer(this);
    connect(progressbarTimer, SIGNAL(timeout()), this, SLOT(upDownloadProgressbar()));
    progressbarTimer->start(1000);
    ui->uploadProgressbar->show();
}

void MainWindow::upDownloadProgressbar(){
    ui->uploadProgressbar->setMaximum(upDownloadMax);
    ui->uploadProgressbar->setRange(0,upDownloadMax);
    ui->uploadProgressbar->setValue(upDownloadProgress);
    if(upDownloadMax * 0.9 < upDownloadProgress){
        progressbarTimer->stop();
    }
}

void MainWindow::upDownloadDataDelayedReset(){
    ui->btnUploadMessages->setEnabled(true);
    ui->btnDownloadMessages->setEnabled(true);
    ui->boxTools->setEnabled(true);
    ui->uploadProgressbar->hide();
}

void MainWindow::upDownloadDataDelayed(bool isUplaod){
    if(ui->inSyncDatasetNameLocal->currentText() == ""){
        QMessageBox::information(this,"Info","The local dataset name shouldn't be empty!");
        return;
    }
    if(ui->inSyncDatasetNameServer->currentText() == ""){
        QMessageBox::information(this,"Info","The server dataset name shouldn't be empty!");
        return;
    }


    QString serverPassword = ui->inServerDBPassword->text();
    QString serverUser = ui->inServerDBUser->text();
    QString serverIP = ui->inServerDBIP->text();
    int serverPort = ui->inServerDBPort->value();
    if(!serverIP.contains(QRegularExpression("(\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}$)"))){
        QMessageBox::information(this,"Info","The server DB IP has a wrong format.");
        upDownloadDataDelayedReset();
        return;
    }
    if (serverPort < 0){
        QMessageBox::information(this,"Info","The server DB Port has a wrong format.");
        upDownloadDataDelayedReset();
        return;
    }

    QString localPassword = ui->inLocalDBPassword->text();
    QString localUser = ui->inLocalDBUser->text();
    QString localIP = ui->inLocalDBIP->text();
    int localPort = ui->inLocalDBPort->value();
    if(!localIP.contains(QRegularExpression("(\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}$)"))){
        QMessageBox::information(this,"Info","The local DB IP has a wrong format.");
        upDownloadDataDelayedReset();
        return;
    }
    if (localPort < 0){
        QMessageBox::information(this,"Info","The local DB Port has a wrong format.");
        upDownloadDataDelayedReset();
        return;
    }

    QSqlDatabase dbLocal = QSqlDatabase::addDatabase("QMYSQL",QString::number(1009));
    dbLocal.setHostName(localIP);
    dbLocal.setPort(localPort);
    dbLocal.setUserName(localUser);
    dbLocal.setPassword(localPassword);
    dbLocal.setDatabaseName(ui->inSyncDatasetNameLocal->currentText());

    if(!dbLocal.open()){
        QMessageBox::information(this,"Info","Couldn't reach local DB!");
        upDownloadDataDelayedReset();
        return;
    }

    QSqlDatabase dbServer = QSqlDatabase::addDatabase("QMYSQL",QString::number(1008));
    dbServer.setHostName(serverIP);
    dbServer.setPort(serverPort);
    dbServer.setUserName(serverUser);
    dbServer.setPassword(serverPassword);
    dbServer.setDatabaseName(ui->inSyncDatasetNameServer->currentText());

    if(!dbServer.open()){
        QMessageBox::information(this,"Info","Couldn't reach server DB!");
        upDownloadDataDelayedReset();
        dbLocal.close();
        return;
    }

    //checks done
    //itterate through tables and start a trhead foreach table
    std::deque<std::thread*> threadVec;
    QSqlQuery tableQuery;
    if(isUplaod){
        tableQuery = QSqlQuery(dbLocal);
    }else{
        tableQuery = QSqlQuery(dbServer);
    }
    if(!tableQuery.exec("SHOW tables")){
        QMessageBox::warning(this,"Info","Couldn't query Dataset for tables! Error: " + tableQuery.lastError().text());
        upDownloadDataDelayedReset();
        return;
    }
    while(tableQuery.next()){
        QString tableName = tableQuery.value(0).toString();

        if(isUplaod){
            threadVec.push_back(new std::thread(upAndDownloadThread,localIP,localPort,localUser,localPassword,ui->inSyncDatasetNameLocal->currentText(),serverIP,serverPort,serverUser,serverPassword,ui->inSyncDatasetNameServer->currentText(),tableName));
        }else{
            threadVec.push_back(new std::thread(upAndDownloadThread,serverIP,serverPort,serverUser,serverPassword,ui->inSyncDatasetNameServer->currentText(),localIP,localPort,localUser,localPassword,ui->inSyncDatasetNameLocal->currentText(),tableName));
        }
    }

    while(!threadVec.empty()){
        std::thread* thr = threadVec.front();
        threadVec.pop_front();
        thr->join();
        delete thr;
    }

    dbLocal.close();
    upDownloadDataDelayedReset();
}


void MainWindow::upAndDownloadThread(QString dbIPFrom, int dbPortFrom, QString dbUserFrom, QString dbPasswordFrom, QString datasetNameFrom,QString dbIPTo, int dbPortTo, QString dbUserTo, QString dbPasswordTo, QString datasetNameTo, QString tableName){
    uint threadID = std::hash<std::thread::id>{}(std::this_thread::get_id());
    QSqlDatabase dbTo = QSqlDatabase::addDatabase("QMYSQL","s_" + QString::number(threadID));
    dbTo.setHostName(dbIPTo);
    dbTo.setPort(dbPortTo);
    dbTo.setUserName(dbUserTo);
    dbTo.setPassword(dbPasswordTo);
    dbTo.setDatabaseName(datasetNameTo);
    dbTo.open();
    QSqlDatabase dbFrom = QSqlDatabase::addDatabase("QMYSQL","l_" + QString::number(threadID));
    dbFrom.setHostName(dbIPFrom);
    dbFrom.setPort(dbPortFrom);
    dbFrom.setUserName(dbUserFrom);
    dbFrom.setPassword(dbPasswordFrom);
    dbFrom.setDatabaseName(datasetNameFrom);
    dbFrom.open();
    QSqlQuery insertQuery(dbTo);

    int n = 0;

    QSqlQuery loadQuery(dbFrom);
    if(!loadQuery.exec("SELECT * FROM " + tableName)){
        qDebug() << "couldn't query table: " << tableName;
    }else if(loadQuery.next()){
        //create table
        QString tableCreationQueryString = "CREATE TABLE " + tableName + "(time TIMESTAMP(3) DEFAULT CURRENT_TIMESTAMP(3)";
        QSqlRecord rec = loadQuery.record();
        for(int i = 1; i < rec.count(); i++){
            tableCreationQueryString.append("," + rec.field(i).name() + " " + Receiver::convertCppTypeToSQLType(rec.value(i)));
        }
        tableCreationQueryString.append(")");


        if(!insertQuery.exec(tableCreationQueryString)){
            qDebug() << "Failed Table Creation: (" << tableCreationQueryString << " ), with error: " << insertQuery.lastError();
        }
        loadQuery.previous();
    }

    progressBarMutex.lock();
    upDownloadMax += loadQuery.size();
    progressBarMutex.unlock();

    while(loadQuery.next()){
        QString insertString("INSERT into " + tableName + " VALUES (");
        QSqlRecord rec = loadQuery.record();

        //do first (time) outside of for-loop (because of ",").
        insertString.append("'" + rec.value(0).toDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz") + "'");
        for(int i = 1; i < rec.count(); i++){
            if(((QString) rec.value(i).typeName()).compare("QString") == 0){
                insertString.append(",'" + rec.value(i).toString() + "'");
            }else{
                insertString.append("," + rec.value(i).toString());
            }
        }
        insertString.append(")");
        if(!insertQuery.exec(insertString)){
            qDebug() << "Couldn't insert in Table (" + tableName + ")! Error: " + insertQuery.lastError().text();
        }
        n++;
        if(n % 1000 == 0){
            progressBarMutex.lock();
            upDownloadProgress += 1000;
            progressBarMutex.unlock();
        }

    }
    qInfo() << "Up/Downloaded "<< n << "Messages to table " << tableName;
    progressBarMutex.lock();
    upDownloadProgress += n%1000;
    progressBarMutex.unlock();
    dbFrom.close();
    dbTo.close();
    return;
}

void MainWindow::customMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    static int countWidgetLog = 0;
    QHash<QtMsgType, QString> msgLevelHash({{QtDebugMsg, "Debug"}, {QtInfoMsg, "Info"}, {QtWarningMsg, "Warning"}, {QtCriticalMsg, "Critical"}, {QtFatalMsg, "Fatal"}});
    QByteArray localMsg = msg.toLocal8Bit();
    QTime time = QTime::currentTime();
    QString formattedTime = time.toString("hh:mm:ss.zzz");
    QByteArray formattedTimeMsg = formattedTime.toLocal8Bit();
    QString logLevelName = msgLevelHash[type];
    QByteArray logLevelMsg = logLevelName.toLocal8Bit();

#ifdef LOGTOFILE
    QString txt = QString("%1 %2: %3 (%4)").arg(formattedTime, logLevelName, msg,  context.file);
    QFile outFile("debug.log");
    outFile.open(QIODevice::WriteOnly | QIODevice::Append);
    QTextStream ts(&outFile);
    ts << txt << Qt::endl;
    outFile.close();
#endif
#ifdef LOGTOCONSOLE
    fprintf(stderr, "%s %s: %s\n", formattedTimeMsg.constData(), logLevelMsg.constData(), localMsg.constData());
    fflush(stderr);
#endif
#ifdef LOGTOWIDGET
    if(type == QtInfoMsg){
        if(countWidgetLog > 10000){
            uiStat->debug_out->clear();
            countWidgetLog = 0;
        }
        countWidgetLog++;
        uiStat->debug_out->append( QString(formattedTimeMsg.constData()) + " " + logLevelMsg.constData() + ": " + localMsg.constData());
    }
#endif
    if (type == QtFatalMsg)
        abort();
}


