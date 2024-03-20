# ME 405 Term Project

## Authors
Jack Krammer and Jason Chang

Assisting professors: 
* Ridgely, John R.
* Refvem, Charlie Thomas

California Polytechnic State University

March 18, 2024

## Description
For the Winter Quarter term of ME 405's lab, we were assigned by Dr. John Ridgely 
to create a **heat-sensing foam dart blaster turret**. Its capabilites include full 
autonomous motion, aiming, and firing. For aiming it would use a lab provided 
thermal infrared camera (model MLX90640) to find its intended target.

Our device was tested against others on Dueling Day where along the ends of a long 
table each opposing device was secured and a single member of each team stood behind 
their respective device. Upon the "duel" beginning, members would move within a 5 
second period, where afterwards they must remain frozen in their chosen position and 
each oppposing device must aim and fire against their opponent within 10 seconds 
afterwards. For further clarification upon project and Dueling Day rules, you may go 
to Dr. John Ridgely's provided HTML file below.
<a href="./src/links/termproj_W24.html" title="term_proj_specs">Term Project Specifications</a>


## Dependencies
This project depends on the MicroPython 
<a href="https://docs.micropython.org/en/latest/library/pyb.html" title="pyb">pyb</a>,
<a href="https://docs.micropython.org/en/latest/library/machine.I2C.html" title="machine I2C">machine.I2C</a>,
<a href="https://docs.micropython.org/en/v1.15/library/utime.html" title="uitme">utime</a>,
<a href="https://docs.python.org/3/library/math.html" title="math">math</a>,
and 
<a href="https://docs.python.org/3/library/gc.html" title="gc">gc</a>
libraries as well as 
<a href="https://github.com/spluttflob/ME405-Support/tree/main/mlx_raw" title="mlx_cam">mlx_cam</a>
and 
<a href="https://github.com/spluttflob/ME405-Support/blob/main/src/cotask.py" title="cotask">cotask</a>
with some associated documentation found 
<a href="https://spluttflob.github.io/ME405-Support/" title="ME 405 documentatin">here</a>.

Documenation and diagrams describing the software we designed for this project can be found
<a href="https://8red10.github.io/ME_405_Term_Project/" title="project documentation">here</a>.


## Hardware Design
This section identifies key components in the hardware design of this project.


### Materials
The materials used to build this project.


#### Micropython Board and Electronics
* 1x L6206 Motor Driver "Shield"
* 1x STM32L476RG Nucleo Arduino
* 1x The Shoe of Brian
* 1x Small Breadboard
* 1x Button
* 1x 0.1 uf Hilitchi Chip Capacitor
* 1x 0.33 uf Hilitchi Chip Capacitor
* 1x 7805A JRC Voltage Regulator

#### Pre-fabricated Components
* 1x Fortnite Nerf Flare Dart Blaster
* 1x Ametek-Pittman PG6712A077-R3 6665 DC Encoder Motor
* 1x 1501MG RC Servo Motor
* 1x 96 teeth 0.5 Module Gear
* 1x 6 in x 6 in Lazy Susan Hardware
* 1x 1 ft Long Shoe String
* 6x Stainless Steel Pan Head Slotted M3 x 25 mm Long x 0.5 mm Pitch Screws
* 2x Stainless Steel Female Threaded Hex M3 x 8 mm Long x 0.5 mm Pitch Standoffs
* 4x Zinc Plated #8 x 1/2 in Machine Bolts
* 4x #8 Hex Nuts
* 3x Stainless Steel Pan Head Slotted M4 x 12 mm Long x 0.7 mm Pitch Screws
* 3x Stainless Steel Hex M4 x 0.7 mm Pitch Nuts


### CAD Model Description
This section the idea behind the design of the main body and associated components.

<div align='center'>

<img src='./src/images/Shaft1.PNG' alt='main shaft' height='300'>

**Figure 1.** When designing the Foam Dart Blaster Turret we centralized it around 
its main axle of rotaion. This rotation would be from the 96 teeth gear. To support 
this design, the central shaft was designed to connect to the 6 inner gaps, as seen 
above.


<img src='./src/images/MainFrame2.PNG' alt='main frame' height='300'>
	
**Figure 2.** In order to power the 96 teeth gear, it needed to connect to the 
chosen motor of this project: the Ametek-Pittman DC Encoder Motor. The motor drives 
a 16 teeth gear that is the same module number as the 96 teeth gear. To ensure 
constant contact between these two gears, the required distance between the two was 
kept in mind when designing the bottom frame of the Foam Dart Blaster Turret. 


<img src='./src/images/MotorHolder1.PNG' alt='motor holder' height='300'>

**Figure 3.** The bottom frame also accounted for the distance gained from the motor 
holder component.


<img src='./src/images/TopPlate1.PNG' alt='top plate' height='300'>

**Figure 4.** In order to prevent the 96 teeth gear from experiencing any excessive 
friction, a lazy susan was incorporated into the design. This meant that the majority 
of the vertical weight from the rotating portion of the Foam Dart Blaster Turret could 
be displaced into the walls of the bottom frame instead. To secure the lazy susan to 
the bottom frame, a plate was made to connect the two components together. 


<img src='./src/images/NerfGunHolder1.PNG' alt='nerf gun holder' height='300'>

**Figure 5.** The main turret was designed to connect to two components; the lazy susan 
and the central shaft. This way it would recieve the torque from the shaft and displace 
its weight into the frame. Alongside connecting to those components, the main turret was 
designed to hold the chosen Foam Dart Blaster (Fortnite Nerf Flare Dart Blaster) and a 
servo motor near the trigger of the Nerf Gun to allow control over when it would shoot.


<img src='./src/images/Assembly1.PNG' alt='full assembly' height='300'>

**Figure 6.** Above is the full CAD assembly of the Foam Dart Blaster Turret from the 
previusly mentioned components. 


<img src='./src/images/Assembly2.PNG' alt='assembly with main shaft' height='300'>

**Figure 7.** To understand how the Foam Dart Blaster Turret better works, it can be viewed 
at two section analysis views. In this first one above, the main turret can be seen being 
connected to the central shaft and the central shaft being connected to the 96 teeth gear 
(of which is represented by a simplified model in the assembly). 


<img src='./src/images/Assembly3.PNG' alt='assembly sectional analysis' height='300'>

**Figure 8.** In this next section analysis view, a clearer image of how the central shaft 
operates is shown. While the 96 teeth gear rotates around the bottom frame shaft, the 
torque it generates is transferred upward to the main turret through the mediated 
connection with the central shaft. Also seen is the small gab between the central shaft 
and the 96 teeth gear. This shows that if fitted properly, no downward force will be 
place onto the 96 teeth gear (other than its own weight), preventing any excessive friction 
at that point. 

<div align='left'>


## Electromechanical Properties

<div align='center'>

| Motor Parameter | Value | Unit | 
|:---------------:|:-----:|:----:|
| Total Rotating Mass | 0.407 | $kg$ |
| Length to Center of Gravity | 0.050 | $m$ |
| Mass Moment of Inertia | 1.125e-3 | $I$ |
| Viscous Damping Coefficient | 3.14e-7 | $b$ |
| Torque Constant | 0.0282 | $K_t$ |
| Back-emf Constant | 0.0282 | $K_v$ |
| Terminal Resistance | 35.0 | $\Omega$ |
| Terminal Inductance | 1.00e-5$*$| $H$ | 
| Motor DC Voltage | 12 | $V$ |

**Table 1.** Characteristics of the motor used in our project.

<div align='left'>

* Length to Center of Gravity is measured from the origin of 
the axis of rotation to the center of gravity.
* Assumed a near zero inductance due to the insignificance 
of the inductance to the load on the turret motor.


## Electronics Design
This section illustrates some of the key custom electronic designs used for this 
project.


### Start Button

<div align='center'>

<img src='./src/images/button_schematic.png' alt='button schematic' height='300'>

**Figure 9.** This illustrates the schematic of the active low button with a pull up 
resistor. The PC2 tag identifies the pin input to the microcontroller.


<img src='./src/images/button_closeup.jpg' alt='button connections' height='300'>

**Figure 10.** For the button we used to trigger the start our device's autonamous 
actions, we used a pull-up resistor set up so that upon button press it would produce 
a low signal to our chosen pin input. From this low signal, we are able to code within 
the micro-controller based around it. The pin we chose to take the button circuitry's 
input was pin PC2.

<div align='left'>

### Emergency Stop Component

<div align='center'>

<img src='./src/images/emergencystop.jpg' alt='emergency stop' height='300'>

**Figure 11.** For the emergency stop we used a red wire that was responsible for 
supplying power to the motors of our project. To connect this red wire to the power 
cable, we used two terminal screws that were secured to our device via zip ties. 

<div align='left'>

### Voltage Regulator for Servo Input

<div align='center'>

<img src='./src/images/servo_schematic.png' alt='servo schematic' height='300'>

**Figure 12.** This illustrates the schematic of the voltage regulator that converts 
the 12V DC supply for the motor down to a managable 6V for the servo's VDD. The PB3
tag identifies the pin output from the microcontroller to the servo signal wire. A
7805A JRC transistor was used to facilitate most of the voltage regulation.

<img src='./src/images/voltageregulator.jpg' alt='voltage regulator' height='300'>

**Figure 13.** In order to supply the correct amount of voltage to our servo motor 
(5-6 V), we used a voltage regulator to reduce our source voltage of 12 V to 5.5 V to 
our servo motor. By following the manuel and connecting two capacitors to its terminal 
ends, we were able to supply our servo motor the proper amount of voltage. 

<div align='left'>


## Software Design
The software running the turret was designed to multitask using a task scheduler
to more efficiently run the turret. The button task monitors the button and waits 
the appropriate 5 seconds before signaling the image task and rotate task to start.
The image task gets a thermal image from the camera and parses the image data 
into a setpoint to send to the rotate task. The rotate task rotates the turret
towards the identified target and fires the turret by actuating the servo. More
information and software documentation can be found 
<a href="https://8red10.github.io/ME_405_Term_Project/" title="project documentation">here</a>.


## Discussion 
This section includes reflection on our process and the results obtained.


### Discussion of Results
For testing, we chose to test each major component of our project seperately first 
before testing the entirety of it. These seperate components were our camera, turret 
(furthermore, the proportional controller), and servo motor. 

For the camera, we would test that, based off the ascii image it produced, if it 
would produce an accurate set point value. For the turret, we would test if it rotate 
in time to a given set point. For the servo motor, we would test if it would rotate 
far back enough to pull the trigger far enough for it to fire a foam dart. After each 
component passed their respective tests, we then moved on to the Dueling Day set up 
for its final test; accurately aiming and firing a foam dart to the target.

When it came to the final test, it had one issue: it would not aim properly. Although 
we tested the camera seperately, the set point value it produced was only 
theoretically correct. However, when it came to testing the device as a whole, it had 
trouble aiming. The resolution to this was editting its aiming algorithim. The way 
it worked before was that it first found the mean heat values of each row, then the 
mean of the mean values. This "mean squared" value was then the row it based its 
aiming from, finding the highest heat value in it as its target. The issue with this 
algorithim was when it would find "mean squared" value for its row selection. At times, 
it would chose a row that did not contain the target's heat signiture. The highest heat 
value in that row would then be non-related to the target, producing an unaccurate set 
point value. To fix this, the algorithim was editted so that instead of finding the 
mean of the mean values, it found the max of the mean values. This meant that by 
choosing the row with the highest heat value average, the intended target's position 
would be accurately captured.

After fixing that issue, our device performed perfectly. Upon its first test after 
resolving the issue, it shoot a foam dart right into Jason's face. After he recovered 
from his injuries, we continued to test the device, becoming furthur confident in our 
turret's capabilities. 


### Discussion on Learnings
Over the course of the creation of this project, there were many obstacles that arose.
Even so, it was from these obstacles that we learned the most. Three major problems
became apparent once assembly began. 

#### Central Shaft Issue
Firstly, the central shaft was too thick as the size of the DC motor was not taken 
into account when modeling. Due to lack of time to print an updated design, the central 
shaft (and connected main turret shaft portion) was sanded down in Mustang60 using a 
vetical belt wood sander. However, due to the heat from sanding, the plastic of the 3D 
print began to melt. So instead of simply grinding down the plastic, the melted layer 
of plastic was pushed to the ends of the shaft (mostly towards the main turret end). 
This provided a small benefit in fusing the central shaft to the main turret together. 
In the end, the central shaft was able to be "sanded" down in size to allow clearance 
for the DC motor. However, if this project were to continue it would be best to shrink 
the shaft to a diamter of 1 and 1/5th of an inch (this is while maintaining the design 
at the bottom of the shaft to connect it to the gear). 

<div align='center'>

<img src='./src/images/meltedshaft.jpg' alt='melted shaft' height='300'>

**Figure 14.** This figure illustrates the melted plastic of the main shaft that was 
the result of our use of the belt sander to allow clearance for the motor.

<div align='left'>


#### Servo Attachment Issue
Secondly, the portion of the main turret that housed the servo motor had walls that were 
too long. Specifically, the servo motor could not fit in due to there being no room for 
the wires to pass through. To remedy this, the back wall was cut down in size. In future 
it would be best to consider the entirety of the servo motor when modeling its housing. 

<div align='center'>

<img src='./src/images/sawedservomotorholder.jpg' alt='sawed servo holder' height='300'>

**Figure 15.** This figure illustrates the resulting servo motor holder after sawing off 
one of the walls to allow room for the wires to pass through.

<div align='left'>


#### Tolerances Issue
Third, the 3D printed base of the project had large tolerances when printed, causing 
the more precise modeling of the base to be ruined. Specifically, the small holes modeled 
house the nuts were too small for them. The intended purpose of these holes were to house 
these nuts along where the screws used to hold down the top plate to the bottom frame 
would go, allowing the screws to thread into the nuts. This would essentially have the 
screws and nuts clamp the bottom frame and top plate together. However, this was not 
possible as said before. Luckily, the project did not create much movement vertically 
along itself, so while the screws we holding the top plate to the bottom frame loose, 
they did perform their main function of preventing the top plate from rotating along 
with the lazy susan. It would be best in future to consider the tolerance range of the 
3D printer when modeling the more sensitive portions of the project. 


## Completed Design
These next images illustrate the final product in action.

<div align='center'>

<img src='./src/images/final_design_back.jpg' alt='final design back view' height='400'>

**Figure 16.** Back view of the final design in action. 


<img src='./src/images/final_design_front.jpg' alt='final design front view' height='400'>

**Figure 17.** Front view of the final design in action. 

<div align='left'>




