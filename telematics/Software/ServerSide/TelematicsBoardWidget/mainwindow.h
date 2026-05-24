#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "receiver.h"
#include <QtSql/QSqlDatabase>
#include <QComboBox>

//#define LOGTOFILE
#define LOGTOCONSOLE
//#define LOGTOWIDGET

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    static inline Ui::MainWindow *uiStat;



private:
    Ui::MainWindow *ui;
    Receiver reci;
    void measurementDelayed();
    void refreshStatusDelayed(int numMessagesAtStart);
    QString escapedCSV(QString unexc);
    void dataSetNameActivatedGeneric(int index, QString password, QString user, QString ip, int port, QComboBox *uiField);
    void refreshDatasetNameGeneric(QString password, QString user, QString ip, int port, QComboBox *uiField);
    void upDownloadDataDelayed(bool isUpload);
    void upDownloadDataDelayedReset();
    static void upAndDownloadThread(QString dbIPFrom, int dbPortFrom, QString dbUserFrom, QString dbPasswordFrom, QString datasetNameFrom,QString dbIPTo, int dbPortTo, QString dbUserTo, QString dbPasswordTo, QString datasetNameTo, QString tableName);
    static inline int upDownloadProgress;
    static inline int upDownloadMax;
    static inline std::mutex progressBarMutex;
    QTimer *progressbarTimer;

    static void customMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg);


private slots:
    void start();
    void stop();
    void openGrafana();
    void loadDBC();
    void readMessageFile();
    void runLocalSQLQuery();
    void deleteLocalDB();
    void refreshStatus();
    void startMeasurement();
    void uploadData();
    void downloadData();
    void settingsDatsetNameActicatedLocal(int index);
    void syncDatsetNameActicatedServer(int index);
    void syncDatsetNameActicatedLocal(int index);
    void refreshSettingsDatasetNames();
    void refreshSyncDatasetNamesLocal();
    void refreshSyncDatasetNamesServer();
    void exportDatasetAsCSV();
    void upDownloadProgressbar();

};
#endif // MAINWINDOW_H
