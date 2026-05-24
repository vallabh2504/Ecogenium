# Server Side code
This code should be run on a laptop in proximity to the router.

It consists of two parts.
1. A Docker setup with a database and an grafana instance
2. A widget for controlling the logging

If you only intend to use the tool and are not interested in developing you should download the latest release version at [Git](https://github.com/Ecogenium/telematicsBoard/releases)

## Docker Setup
We use a docker-compose file inorder to start a consistent environment on every system. It contains a MySQL-Database and a Grafana-Instance.
The same docker-compose (with slightly adjusted port values) is also used in the portainer on our server.
### Running:
1. Install and run Docker Desktop
2. Then run "./docker_rebuild_and_start.bat" the first time. And every start after that run "./docker_start.bat".
3. To shut down the docker setup run "./socker.stop.bat" (If you just stop docker, that also works. Then the Setup will automatically run the next time you start docker.)

## Widget
### Running: 
The Widget will be provided as a Zip folder. Extract the folder, but keep the files inside. Then run the ".exe".
For the Widget to work you need to connect to the Wifi of the onside Calypso (The one marked as "server" with tape).
The SSID is "EcogeniumServerSide" and the password is the standard ecogenium password.

### Usage:
0. Connect to the "EcogeniumServerSide" wifi and start the docker setup
1. Set the settings in the lower left corner for your local database. (Should allready be correct, if you use the standart settings) Here you can also load a new DBC file, but a working DBC file should already be loaded at startup. [You can also replace the one within the extracted Zip file]
2. Set a Datasetname you want to log to. This should be a meaningfull name per logging session
    -> you can choose an already existing one or add a new one by typing and hitting enter
3. Local IP and Port should usuallly stay the same
4. Now you should be ready to log data. For this press "Start Receiving".
5. You can check if you receive messages, by using the "Status" area in the top right.
6. You can stop logging by pressing "Stop Receiving"

### Synchronization
While connected to the VPN you can upload data to a server inorder to share it with the team.
1. Set DB parameters in the lower rigth corner (Should already be correct)
2. Set the Datasets you want to send to/from (Again you can create a new one by typing and hitting enter)
3. Then you can choose upload to send data to the server or hit download to get data from the server to your local DB.    

!!!This may take quite some time without any response from the program

### SQL Query
This button runs a query of your choosing on the dataset specified in the settings.

!!! This can do real damage, only use this if you know what you are doing

### Delete Local Dataset
This button deletes the currently selected local dataset permanently!!!!

### Export Dataset as CSV
Lets you export the data as csv for doing calculations with it

### Load messagese from file
Loads the messages of a file (For example from the SD card). Messagees will be stored in the currently selected Dataset. 

!!! This may take quite some time without any response from the program

### Grafana 
First you need to open your local Grafana-Instance you can do this via the button in the App or via this link [LocalGrafana](127.0.0.1:3000/).

Here you now need to add a new Data Source. Do the following:
1. Data sources
2. Add new data source
3. choose mysql
4. leave settings as they are except for: Host Url=db:3306, USername=root, Password=ecogenium4thewin
5. Hit save & test

Because the data is split into Datasets per sessions Dashboads should use a Variabel for the Dataset. You can find an example you can copy on the Grafana hosted on the server. [Dashboard](http://10.8.0.1:3025/d/nbkgkgujg/ecomarathonview2025?orgId=1)
If you want to share a Dashboard do it as follows: Open Dahsboard -> Share -> Export -> check "Export for sharing externally" -> use json or file -> Switch grafana instance -> Dashboards -> New -> Import -> paste file/json -> enjoy  (=

### Trouble shooting
For so far unknown reasons sometimes the router needs to be restarted for data to be received. This is only a problem at the start. Once data is being recieved this problem doesn't occur. !!!Check before run


### Devloping:
The widget is programmed using QTCreator using QT6.8.2.
It requires the qmysql drivers to be installed in the drivers folder. (You can find tutorials and precompiled qmysql versions online)
The Widget also requires you to install "QTSerialBus" via the QtMaintanance tool for the CAN classes. (This should be installed together with QTCreator)

#### Logging
For Debugging the code it can be useful to set different logging levels. This can be done in the mainWindow.h by commentin in/out the following lines:

#define LOGTOFILE

#define LOGTOCONSOLE

#define LOGTOWIDGET

You can use all,some or none of them.

#### Deploying Widget
1. Build in release mode
2. copy .exe into new folder
3. open QT 6.8.2 MingGW from the start menu
4. navigate into the folder
5. execute "windeployqt TelematicsBoardWidget.exe"
6. copy in the mysql.dll, libcrypto-3-x64.dll, libssl-3-x64.dll and a dbc file called "messages.dbc "
7. be happy



# Emergency Solution
During SEM25 We needed to use a laptop with Busmaster running to log our data. To convert Busmaster log fiels to a format readable by the Widget you can use "LoggingBusmasterToSDFile.py"