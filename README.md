# Virtual truck trailer formation control of multiple mobile robots

This code is free to use under GNU GPL.
This is an implementation of a formation controller for "n" number of robots. It runs on AdeptRobot's API Aria Library under Ubuntu 12.04
This code can be used on any type of differential mobile robots which has this API.

Running in a simulator:
------------------------

1.Open Mobilesim simulator of AdeptMobilerobots.

2.Load the map file of your choice.

3.Open a terminal and compile the cpp files.

4.To compile the server pgm, type: 
`g++ -o servertruckcircleline -I/usr/local/Aria/include -I/usr/local/Aria/ArNetworking/include -L/usr/local/Aria/lib servertruckcircleline.cpp -lAria -lArNetworking -ldl -lrt -lpthread`
An executable will be generated.

5.Run the exe as `./servertruckcircleline`. This executable default connects to the simulator.

6.To compile the client pgm, type:
`g++ -o client -I/usr/local/Aria/include -I/usr/local/Aria/ArNetworking/include -L/usr/local/Aria/lib clientserverttppcirleline.cpp -lAria -lArNetworking -ldl -lrt -lpthread`
An executable will be generated.

7.Run the exe as `./client` 
This executable connects to the server running in the simulator.

The map from MobileSim resembles like this and a zoomed view of the Amigobots moving could be seen:
![image](https://user-images.githubusercontent.com/2436747/47859289-4be42d00-ddee-11e8-9e06-3f4e957e3470.png)


********************************************************************************
Running in real P3DX robots:
----------------------------
1. Connect the laptop with one robot using USB to RS-232 pin.
2. Connect the other laptop to the robot using USB to RS-232 pin.
3. Make sure that both the laptop connected via wifi in the same network.
4. Ping each other laptop using their respective IP addresses.
5. One robot runs the server pgm and the other robot runs the client pgm. Switch on the robot and make sure the robots are connected to the laptop.
6. Now after compiling the exectables in their respective laptops, run the executable from the terminal.
7. Run as: `./executable name -rp portno` (guidelines are provided in thesis as well as in p3dx manual)
8. Each `exe` connects to the robot by default.
9. Make sure the simulator is off. The program connects to the simulator by default. 
10. Terminating the exe disconnects the program from the robot.

*********************************************************************************
Running in real amigobots:
--------------------------
1. Only one laptop is needed to run both the server and client pgm for two amigobots. 
2. Connect the two amigobots and the laptop on the same network through wifi.
3. Ping both the robots using their respective IP addresses from the laptop.
4. One robot runs the server pgm and the other robot runs the client pgm. 
5. Now after compiling the executables in the laptops, run both the executables from the terminal.
6. Run as: `./executable name -rh ipaddress` (guidelines are provided in thesis as well as in p3dx manual) for each of the robot.
7. Each exe connects to the robot by default.
8. Make sure the simulator is off. The program connects to the simulator by default. 
9. Terminating the exe disconnects the program from the robot.

****************************************************************************************

