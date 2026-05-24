# Car Side
This code is run on a BUS-Interface plugged into the Telematics-Board. It receives CAN-Messages, stores them on a SD-Card and sends them via uart to the Calypso-Wifi module. 

## Usage
If you just want to use the Telematics Module you can use the precompiled version found in the latest release [Git-Release](https://github.com/Ecogenium/telematicsBoard/releases/tag/V0.4).

You need to comment out the line "managementController->createRunnerTask();" in the main.cpp for this code to work as intendet. The can-message lookup loop has been moved to the component.

## Developing
This code is meant to be executed using Atava therefore you should follow the Atava documentation for furhter specifics regarding porgramming. This code requires some additional Atava features that should by now be included int the main branch. These are, this PR (https://github.com/Ecogenium/Atava/pull/28) and a modified arduino library (Unsure what has been changed. If additional info required ask Jan Gehla).

### Hints
This code requires the telematics component to run on the same core as the management controller! Otherwise the CanBus may overwrite a pointer that is used by this code.
This code requires a specific atava version, that can be found as a branch in the Atava repository ("Atava-for-telematic"). (May be adjusted by a fututre developer)

### Time
This code needs to be compiled with the correct time setting. In the "TelematicsComponent.cpp" is a define "USE_SERVER_TIME". 
If it is declared this code will only log relative time after boot. When data is read live this is adjusted with the server time. If it is read from a file it starts in the morning of january the first.
If it is not declared the code will wait for a GPS Signal inorder to get the actual time from that. 
This system should give a useable time for both live and recorded data.

### Testing
For testing Busmaster and the Peak-CAN are basically required ^^.