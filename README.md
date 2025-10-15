# OpenHardware
In this repository I will upload open hardware projects made by me from time to time.

## Rover Speed Controller
In this project I have developed a rover speed controller using a multicore embedded system (ESP32), the core 0 is managing the communication tasks (decoding the wheels speed reference given by a RF receiver) and the core 1 is managing the control algorithm.

The control algorithm used is a "Linear-Quadratic Regulator" (LQR), it is an State-Space Controller (the rover is a MIMO system) based on "Optimal Control Theory". The communication between the cores is managed by a Real Time Operating System (FreeRTOS). 

Inside the folder "RoverSpeedControl" you can find the Matlab/Simulink files to modelate the system, design and tune the LQR controller, the schematic to understand how the hardware is designed, gerber files to manufacture a copy of the hardware, Bill Of Materials (BOM) file to buy the electronic components and source files to program the ESP32 with the algorithms to run it as I did.

### Images of the project
Pictures of the device finished.
![Hardware](RoverSpeedControl/img/Planta_Perfil.png)
High-Level Architecture System Diagram.
![HLAR](RoverSpeedControl/img/Hardware_Sch.png)

Picture of the "Computer-Aided Design" (CAD) 3D Model given by Kicad EDA.
![CAD](RoverSpeedControl/img/CAD.png)
Linear-Quadratic Regulator controller, from theory to implementation.
![LQR](RoverSpeedControl/img/Controller.png)



## Differential Probe
In this project I have developed a differential probe for my oscilloscope (ADP3450). The device is capable of measuring a voltage range between +- 500V, the user can change the scale from x100 to x10 (reducing the full scale range to +- 50V). The bandwith of the probe is around 10 MHz. Inside the directory people can find the schematic, gerber files and LTspice models to analyce the analog circuitry.

### Images of the project
3D Model given by Kicad EDA.
![3D_Model](https://github.com/JMcordobamendez/OpenHardware/assets/79694677/1e5c2da4-005e-4e99-b5d7-385fe00ee8c4)
The device in reality after the manufacturing process.
![Real](https://github.com/JMcordobamendez/OpenHardware/assets/79694677/1922a0cd-cd09-43c2-b694-89d4b15a0b88)
Simulated behaviour using the LTspice software.
![Simulation_Result](https://github.com/JMcordobamendez/OpenHardware/assets/79694677/7e137dad-4113-4fb5-b943-eaaa4a9e37b5)
Real behaviour doing a test with the ADP3450.
![-20dB](https://github.com/JMcordobamendez/OpenHardware/assets/79694677/b32e05de-6dd4-4159-a5e1-706a3818a937)
Test setup.
![Test_Real](https://github.com/JMcordobamendez/OpenHardware/assets/79694677/a580dd4b-7fc1-4264-9605-53a9642e52a9)

