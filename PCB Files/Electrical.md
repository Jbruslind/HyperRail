---
layout: page
title: Electrical
---

## Overview 
The electrical subsystem was designed with 3 key elements in mind: 

* Plug and Play operation 
* System Reliability 
* Expandable operation 

Plug-and-Play operation was taken to mean minimal requirements for hookups and other connections and to prioritize standard connectors that are easily available from commercial outlets. System Reliability was necessary for operation under a wide variety of conditions (wide Vin range, built-in protections, and temperature/humidity resistance) and to limit repairs by the user. Finally, it was assumed that this system should be able to accommodate other configurations than just a 3D CNC gantry and should have features that can be expanded for later development. 

## System Diagram 

![System Diagram](assets/Electrical_assets/SystemDiagram_Electrical.png)

A system diagram was created to help design the infrastructure and architecture for the overall system. In this case my idea was to approach every operable component as a "node" which could act as an independent unit. The reasoning being that I could assign each "node" to team members and give them a black box explanation for each. As an example, the motor node circuit needed to operate using the following parameters: 

+----------------------+-----------------------------------+
| Parameter            | Valid ranges                      |
+======================+===================================+
| Vin Range            | 12 - 30V                          |
+----------------------+-----------------------------------+
| Motor Current (rms)  |  2A                               |
+----------------------+-----------------------------------+
| Signal Inputs        | Step, En, Dir (STEP/DIR control)  |
+----------------------+-----------------------------------+
| Step Frequency       | < = 5Khz                          |
+----------------------+-----------------------------------+
| Microstepping        | 1x - 16x                          |
+----------------------+-----------------------------------+
| Input Interface      | RJ45 8 pin                        |
+----------------------+-----------------------------------+

By providing the specification and known inputs / outputs I could keep each node organized and have the ability to assign its development to any members that were working under my at the time. 

The main blocks in the system diagram that needed to be developed were: 
* Integrated Stepper Driver (Motor Node) 
    * Motor controller using DIR/STEP controls (or eventually UART)
* ESP32 GRBL Controller 
    * This node was meant to be the main firmware controller which would determine the signals to be sent to each motor to tell it where to move. This could be abstracted to basically just run any CNC control firmware such as Marlin, GRBL, or LinuxCNC. While researching it was determined that the ESP32 GRBL firmware met all the requirements and more with little to no development necessary (see the ESP32 GRBL section)

## Motor Nodes

![Motor Node](assets/Electrical_assets/IntegratedStepDriver.png)

The Motor Node PCB was meant to be an independent solution for stepper motor control. The idea was there could be many motor "nodes" which could be controlled using a known standard protocol. The controlling board could be adaptable to a variety of stepper motor types and would be easily mounted to the motor itself. 

![Motor node mounted](assets/Electrical_assets/Integrated_Driver_Demo.png)

The node has 3 I/O connectors for different purposes. The RJ45 connector (commonly used as an Ethernet connector) is used for Power and Control to the Stepper Driver IC: 

![Ethernet Highlight](assets/Electrical_assets/Integrated_Driver_Top_RJ45.png)

The pinout diagram can be found on the schematic pdfs which are included within the documentation under "HyperRail\PCB Files\Integrated Stepper Motor Driver-(fab,pcba)\PCBA-Integrated Stepper Motor Driver"

### Stepper IC 

![Pinout Schematic](assets/Electrical_assets/Stepper_Driver_TMC2209.png)

* Red (bottom right) | RJ-45 Pinout   
    * This Pinout is used for Power input and control signals to the TMC2209 driver IC 
* Pink (Top) | Sense Resistors 
    * The sense resistor selection is incredibly important to the TMC driver current limits. If a larger / smaller motor is required, then these should be changed to reflect that using the recommended lookup table (taken from TMC2209 Datasheet, section 8): 
    ![](assets/Electrical_assets/Rsense_TMC2209.png)
* Yellow | VRef Selection 
    * The VRef circuit uses a simple voltage divider to maintain "soft" current limits. The idea is that the sense resistors set the "hard" current limits in that they are hardware defined whereas the voltage divider circuit allows the user to customize "soft" limits. A radial potentiometer is used for adjustable resistance and a silkscreen diagram is provided to indicate the direction to turn to increase motor current. 
* Blue | Motor Phase Outputs 
    * The output for the motor phase currents from the Stepper Driver IC is a simple 4 pin screw terminal. This allows users to use a variety of stepper motors without having to find / assemble special connectors. All that's necessary is to determine the phase diagram, strip the wires and plug in. 
    
![](assets/Electrical_assets/Integrated_Driver_Top_DriverIC.png)

### Power Conversion and Status LEDs

![](assets/Electrical_assets/Integrated_Driver_Power.png)

* Red | Buck Converter 
    * Main buck operating circuit which are simply an input and output capacitor and the integrated module. 
* Blue | Reverse-Protection Diode 
    * Prevents negative spikes or reverse current 

Power Conversion is done using an integrated buck controller / switch, the TI  [LMZM23601V5SILR](https://www.ti.com/product/LMZM23601). This is a wide Vin range (4 -> 36V) buck converter which will step down the power input (expected 12 - 24V) to logic level (5V) for the Stepper Driver. Not much current is expected for this application, the estimated current draw by the driver for logic level is 10mA max: 

![](assets/Electrical_assets/Tmc2209_supply.png)



![](assets/Electrical_assets/Integrated_Driver_Top_Power.png)

There are 2 physical test points which were designed for multimeter probes to easily measure the voltage output during manufacture and testing. 
## ESP-32 Controller

The ESP-32 Controller was designed to be the liaison between motion requests by the ROS software and the rest of the machine. The board itself is computationally powered with an ESP32-WROOM module (either through a daughter dev-board or embedded on the board) running the [GRBL_ESP32](https://github.com/bdring/Grbl_Esp32) firmware. A key aspect of using the GRBL style firmware over other flavors (such as Marlin or otherwise) is that GRBL supports the ESP32 with no perquisites and can support a Web-UI natively for manual control. 

![GRBL Controller](assets/Electrical_assets/ESP32GRBLController.png)



## Magnetic End Stops

## Main System

