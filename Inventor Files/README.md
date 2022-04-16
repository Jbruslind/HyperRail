---
layout: page
title: Mechanical
---

## Overview 

The Mechanical system was designed with modularity and easy assembly in mind. In the future, the system may be deployed in different areas/environment and may require an increase or decrease in dimensions.  In addition, the system will be used by users with little to no prior knowledge of how it was designed or any institutional knowledge of how it was originally assembled. With these aspects in mind, the mechanical system was created with defined "sub-assemblies" in mind so that each sub-assembly is easy to understand and assemble and can integrated into the full system seamlessly. This also allows for the option to upgrade specific sub-assemblies as needed if new ideas or designs are discovered (so long as the base needs are met). 

![Full System](https://raw.githubusercontent.com/Jbruslind/Jbruslind.github.io/main/assets/Mechanical_assets/FullSystem.png "Full System")
### Design Objectives 
The main design objectives for this system were: 
1. Modular Sub-Assembly Structure
    * Drop-In Replacements 
    * Iteration 
    * Dynamic Length / Width 
2. Reliable Linear Motion 
    * Long Distance Motion 
    * Self-Contained 
3. Standard Materials
    * Corrosion Resistant 
    * Widely Available 
4. Standard Fasteners
    * Only 1 Allen Key Required for Assembly 
5. Limited Customized Parts
    * Minimize User Manufacturing
## X Carriage Subassembly

![Carriage Subassembly](/assets/Mechanical_assets/Carriage_Assembly.png "Carriage Subassembly")

The X Carriage Subassembly is the main motion platform for the HyperRail system. The assembly should allow for smooth motion along the lower 4060 T-slot track while also allowing the motor mechanism to interface to the belt track. 

The carriage assembly borrows ideas from the [Bell-Everman](https://www.bell-everman.com/products/linear-positioning/servobelt-linear-sbl)[^1] servo setup in which a static belt is used on the tracks and interfaces to a dynamic belt driven by a motor / pulley. A rack and pinion system would also work the same way (with more durability) however the realative costs were prohibitive for a system on this scale. For reference, the average cost per meter for the rack on a rack and pinion is ~$70/m whereas for this style, the belts are only ~$15/m. The projected user needs are for the system to extend up to 90ft or 30m making the rack and pinion very expensive. 

<img src = "/assets/Mechanical_assets/BellEverMan.jpg" width ="525"/> <img src="/assets/Mechanical_assets/compareBellServo.png" width="425"/> 

One of the big advantages to this style of mechanism is the ability to easily extend the track length to various sizes without needing to purchase specially made belts/tracks. This allows the overall system to maintain "modularity" with the x-direction, which is aimed to be the longer of the axes (x length >> y, z). If the user wants to add more length, they can easily attach another section of track (4060 with static belt attachment) to the original axis and align the track pieces together so the dynamic belt is able to mesh smoothly (see the Add-On module section). 

The Carriage itself is composed of 3 pieces of 2040 aluninum T slot extrusion which make up the main structural body or chassis. A long piece is used on the side with the vertical beam to give it added support and a smaller piece is used on the side with the motor. 

![Smaller Side](/assets/Mechanical_assets/Carriage_Assembly_motorside.png)

Individual plates with delrin wheels are used for rotational motion and structural stability. 

## X Rail Assembly

![X Rail](/assets/Mechanical_assets/Side_Rail_Assembly.png) ![Side Profile X Rail](assets/Mechanical_assets/Side_Rail_Side_Assembly.png)

The X Rail Assembly utilizes the carriage shown above combined with a track and vertical 4040 T-slot beam. The track itself is made of 4060 aluminum which was chosen for symmetry and stability (increased mass provides a good counterweight). In order for the mechanism to be stable, the carriage base needs to equally distribute mass where the most mass comes from the vertical beam + upper axis and the motor + planetary gearbox. Therefore the X Rail Assembly cantilevers the 2 largest masses against each other so the center of gravity is relatively centered within the X carriage.


The assembly is supported by a foot subassembly which can be added and distributed as necessary to support the system in different environments. 

![Bottom Foot](/assets/Mechanical_assets/BottomFoot.png)

These feet allow for small height adjustments on either side through an M6 bolt embedded in the design which can be useful for outdoor use on rocky / dirt environments.

The pulley attached to the motor can be adjusted using the right angle brackets and can be used to provide tension to the belt system (height adjustment shown below). The whole mechanism can then be moved using the dynamic belt rolling along the static belt 
![X Rail Movement](/assets/Mechanical_assets/MotorAdjust_Movement.gif)

The static belt layer is comprised of HTD 5mm, 25mm width polyurethane belt with steel cord reinforcement from [PolyBelt](https://shop.polybelt.com/20-5M-Open-End-Belt-Roll-Polyurethane-with-Steel-Cords-B20-5M-MPS.htm). Polyurethane was chosen for its durability and resistances to outdoor environments and steel cord was chosen for its price point. Kevlar reinforcement was another option which had a better tensile modulus (lower stretching over time) however this was determined to not be a huge factor since the static belt wouldn't have large segments stretching at the same time (the stress is localized to each individual point). At the same time, Kevlar has larger thermal fluctuations [^2] which could cause problems during outdoor use in freezing / hot temperatures.

This profile is standard across many suppliers but PolyBelt was chosen for their price point and US location. In addition they had a matching closed belt profile which was the right size (6mm) and length (635mm) for the dynamic belt system.

## Top Frame Subassembly

The top frame subassembly was meant to serve as the Y-axis for the overall system and is comprised of a main 4040 T-slot extrusion paired with the same Bell-Everman style linear motion as the X-Carriages. 

![](/assets/Mechanical_assets/Top_Rail.png)

The hollow channel within the T-slot extrusion serves as a pathway for wiring from one side of the system to the other, specifically for the motor and magnetic endstop wiring. These wires are static with respect to the gantry and so can be threaded through without any torsion later. 

![](/assets/Mechanical_assets/Top_Rail_Channel.png)

## Y Carriage Subassembly

## Ebox Subassembly

## Main System

## References
[^1]: Bell-Everman, Inc. (2016, October 19). New ServoBelt™ LoopTrack Linear Stage. Retrieved from https://www.bell-everman.com/products/linear-positioning/servobelt-linear-sbl
[^2]: When to consider Kevlar reinforced belts for linear motion applications. (2019, July 05). Retrieved from https://www.linearmotiontips.com/when-to-consider-kevlar-reinforced-belts-for-linear-motion-applications