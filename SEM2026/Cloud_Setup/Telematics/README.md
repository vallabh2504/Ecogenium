# Telematics Board
This project contains the hardware files for Altium, code for on the BUS-Interface as well as the code used to run the server.

A higher-level documentation and a season description can be found on [qWiki](https://ecogenium.qwikinow.de/content/cb4fe812-0402-4ff7-80c1-0b5effc30064?title=2425-telematics-board).

Things that need fixing are documented in  git issues (https://github.com/Ecogenium/telematicsBoard/issues).

## Hardware Documentation
- Documentation: [Git](https://github.com/Ecogenium/telematicsBoard/tree/master/Hardware)
- Shematics: [Git](https://github.com/Ecogenium/telematicsBoard/tree/master/Hardware/Shematics-Docs)

## Software Dokumentation
- Car-Side: [Git](https://github.com/Ecogenium/telematicsBoard/blob/master/Software/CarSide/README.md)
- Server-Side: [Git](https://github.com/Ecogenium/telematicsBoard/blob/master/Software/ServerSide/README.md)

## Setup
Setting up logging requires a few components:
1. telematicsboard (Should be in the car)
2. Small Wifi-Antenna (in Red Box by Würth) + connector cable (In Car. If not, then in Red Box)
3. SD-Card (inside Telematicsboard or in the Red Box)
4. SD-Card reader (in the Red Box)
5. Wifi-Router [red dev-board with "Server" written on it] (in the Red Box)
6. Directed Antenna (In the Red Box)
7. Extendable Tri-Pod (in the corner of the electrical room next to the electrical cupboards)
8. Mount for on the Tri-Pod (brown Box on the highest shelf of the electrical cupboard)
9. Powerbank (bring one yourself)
10. Cable Ties
11. Micro-USB cable (There are 2 in the Red Box)
12. For longer usage the EcoFlow is required to keep the laptop charged

Steps to follow for setup (Some of these are further explained in the Software Documentation):
1. Upload code to the Telematicsboard
2. Attach Telematicsboard to the car (Ceiling of the cockpit)
3. Attach antenna through the roof of the car
4. Clear the SD-Card and slide it into the Board
5. Attach the mount to the Tri-Pod
6. Connect the Directed Antenna with the Router.
7. Squeez the Directed Antenna into the mount on the Tri-Pod.
8. Attach the powerbank on the Tri-Pod using the cable ties or tape
9. Power the Router using the USB cable
10. Use a laptop nearby to connect to the Wifi hosted by the router
11. Start Docker and the Telematics Software


## Contact
If any questions regarding this project come up contact Kai Welsing:
- kai.welsing@googlemail.com
- 01788375041




## Connection via Hotspot
Setup the Calypso to connect to the hotspot (expalined in hte hardware documentation)
Setup the Calypso to send all data to the ecogenium server 195.201.134.169
Connect your PC to the VPN
SSH into the server and execute "nc -u -l -p 5001 | nc -u localIP 5001", where localIP is the IP shown in the VPN tool
