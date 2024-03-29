'''!
@mainpage Introduction
This is the software documentation for the ME 405 final project.
\n\n


@section sec0 Software Overview
The software running the turret was designed to multitask using a task scheduler
to efficiently run the turret. There are three tasks: button task, image task, 
and rotate task. The button task monitors the button and waits 
the appropriate 5 seconds before signaling the image task and rotate task to start.
The image task gets a thermal image from the camera and parses the image data 
into a setpoint to send to the rotate task. The rotate task rotates the turret
towards the identified target and fires the turret by actuating the servo.
\n\n


@subsection ssec0_0 Software Organization
This project is organized such that the `main.py` file imports necessary driver
classes. This file also creates the turret generator class that contains the 
individual task generator functions as methods of the turret generator class 
to allow for ease of use of shared variables. When run as the main file, 
`main.py` initializes the tasks and continuously runs the task scheduler until
a `KeyboardInterrupt` is caught.

The `encoder_reader.py` file contains the class to interface with the encoders
built into the DC motors used in this project as well as the ME 405 lab.

The `mlx_cam.py` file contains the class to interface with the MLX90640 thermal
infrared camera. This camera is used to obtain information of the target's 
location to derive a setpoint to aim the turret to.

The `motor_driver.py` file contains the class to interact with and actuate the
motor through the L6206 Motor Driver Hat attached to the STM32 board. This 
motor is what is used to rotate the turret.

The `proportional_controller.py` file contains the class that can run a 
proportional controller with a changeable position gain and derivative gain. 
This class is initialized to read from the encoder and actuate the motor.

The `servo_driver.py` file contains the class to set the position of the servo 
that pulls the trigger of the turret.

The `test.py` file contains testing functions and classes useful for solving 
issues and learning more about this project. The `main.py` file does not 
`import` the `test.py` file, this is simply included for extra information 
about some of our testing processes.
\n\n


@section sec1 Diagrams
This section includes the task diagram and state diagrams to provide more 
information on the control of the turret.
\n\n


@subsection ssec1_0 Task Diagram

<img src='task_diagram.png' alt='task diagram' height='400'>

Figure 1. This figure illustrates the task diagram for the turret process.
The assigned priorities are based on the difference in time it takes for each 
task to run. As the image task takes the longest, we didn't want it to block
all other tasks, so we made it the lowest priority and with a frequency that 
was tested to allow an image update almost every task run. As the frequency of 
the rotate task function is also the frequency the proportional controller is 
run, this frequency was not as flexible, but 50Hz was chosen to ensure 
quality behavior without losing too much efficiency. For the button task, 
this was set to the highest priority because if it did interrupt other tasks,
it would be so short that there would be minimal exclusion. The button task
also runs at the highest frequency to detect even extremely short button 
presses. The `start_flag` and `image_flag` variables shared by the button task 
enable the rotate task and image task, respectively. The shared 'setpoint' 
variable is shared by the image task as the setpoint is derived from thermal 
image data. The shared 'setpoint' variable is recieved by the rotate task as
this is the value to run the proportional controller with. This sharing 
optimizes cooperation and division of labor between tasks.
\n\n


@subsection ssec1_1 State Diagrams

@b Rotate @b Task

<img src='rotate_state_diagram.png' alt='rotate state diagram' height='300'>

Figure 2. This figure illustrates the state diagram for the @b rotate @b task
of the turret process. The `start_flag` variable ensures that the turret will
not rotate unless the button task has allowed it to. The threshold is to 
ensure that the turret is in the desired position before moving to the FIRE 
state and pulling the trigger. After this FIRE state, the `start_flag` variable
is reset so the turret won't shoot twice. 
\n\n

@b Image @b Task

<img src='image_state_diagram.png' alt='image state diagram' height='300'>

Figure 3. This figure illustrates the state diagram for the @b image @b task
of the turret process. The `image_flag` variable ensures that the turret will
not capture images until after the button has allowed it to. The `image` 
variable check ensures that there is an image available to parse before 
moving to the PARSE state. After this PARSE state, the `image_flag` variable
is reset so the camera will not waste computational power capturing another 
image after it already has one.
\n\n

@b Button @b Task

<img src='button_state_diagram.png' alt='button state diagram' height='300'>

Figure 4. This figure illustrates the state diagram for the @b button @b task
of the turret process. The `SW1` variable represents the start button and holds
the button task in the CHECK state until the button is pressed. The INIT state
initializes variables required at the beginning of the turret process (i.e. 
resetting the encoder, setting the initial setpoint, turning on the builtin
LED for user information). The WAIT state waits the required 5 seconds before 
setting the `start_flag` and `image_flag` variables high to start the rotate 
task and image task. Once in the DONE state, the button task does nothing,
waiting for a user to cause a `KeyboardInterrupt` to exit the turret process.
\n\n


@section sec2 Schematics
This section includes schematics of some of the custom electronic design
used for this project.
\n\n


@subsection ssec2_0 Button Schematic

<img src='button_schematic.png' alt='button schematic' height='300'>

Figure 5. This figure illustrates the electrical schematic designed to read
an active low button to start the turret process. PC2 is the signal input 
to the microcontroller.
\n\n


@subsection ssec2_1 Servo Schematic

<img src='servo_schematic.png' alt='servo schematic' height='300'>

Figure 5. This figure illustrates the electrical schematic designed to 
convert 12V DV to ~6V with a 7805A JRC transistor to supply the servo 
used to pull the trigger on the turret. PB3 is the signal output from
the microcontroller to control the PWM to the servo and therefore 
position of the servo.
\n\n

'''
