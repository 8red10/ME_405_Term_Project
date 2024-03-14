'''!
@file main.py
@brief      Main file for the ME 405 Term Project.
@details    Fires a Nerf dart at a detected thermal signature. Utilizes 
            a state machine to control the overall process. Waits for a 
            push button input to start. After button input, waits 5 seconds
            for opponent to position themselves. Then reads an image from a
            MLX90640 thermal camera and parses the location of the opponent.
            Then aims the turret towards the opponent with the use of a 
            propotional controller. Then actuates a servo to press the
            trigger to fire the Nerf dart. 
            
            State Diagram:
                Start -> IDLE state -> WAIT state -> LOCATE state -> 
                PARSE state -> ROTATE state -> FIRE state -> RESET state
            
            IDLE state:     waits for the push button to start the process.

            WAIT state:     waits 5 seconds for opponent to position 
                            themselves.

            LOCATE state:   captures an image from the MLX90640 thermal 
                            camera.

            PARSE state:    obtains the opponent location from thermal image
                            data.

            ROTATE state:   rotates the turret to the desired location.

            FIRE state:     actuates a servo to pull the trigger on the 
                            Nerf gun.
            
            RESET state:    waits here until positions of the turret are
                            manually reset.


            Camera qwiic connection: 
            Black wire      = ground
            Red wire        = 3V3
            Blue wire       = SDA (I2C bus 1 = PB9; I2C bus 2 = PB11)
            Yellow wire     = SCL (I2C bus 1 = PB8; I2C bus 2 = PB10)

                            
@authors    Jack Krammer and Jason Chang
@date       12-Mar-2024
@copyright (c) 2024 by mecha04 and released under MIT License
'''

# imports here
from machine import I2C
import pyb
import utime
import motor_driver
import encoder_reader
import proportional_controller
import mlx_cam
import servo_driver
# from array import array
import math
import cotask
import task_share
import gc

# global constants
ENCODER_COUNT_PER_REV   = 98218 #16384 # TODO
MOTOR_CONTROL_INTERVAL  = 10        # milliseconds
MOTOR_CONTROL_PERIOD    = 2000      # milliseconds
MOTOR_CONTROL_POINTS    = MOTOR_CONTROL_PERIOD // MOTOR_CONTROL_INTERVAL
MOTOR_ACTUATE_THRESH    = 15        # duty cycle %      # TODO
ROTATE_POS_THRESH       = 100       # encoder counts    # TODO
THERMAL_LIMITS          = (0,100)
BUTTON_TASK_INTERVAL    = 10        # milliseconds
IMAGE_TASK_INTERVAL     = 160       # milliseconds
SERVO_START_POS         = 2.0       # pulse width (milliseconds)
SERVO_PULLED_POS        = 1.65      # pulse width (milliseconds)

# global wait time constants
WAIT_TIME               = 5000      # milliseconds
IMAGE_WAIT_TIME         = 50        # milliseconds
SERVO_WAIT_TIME         = 2000      # milliseconds

# global geometric placement variables
PERP_DIST_CAMERA_TO_TARGET      = 9 #8     # feet
PERP_DIST_TURRET_TO_TARGET      = 17 #14    # feet
CAMERA_FOV_ANGLE                = 55    # degrees

# turret button FSM states
CHECK   = 1
INIT    = 2
WAIT    = 3
DONE    = 4
# turret rotate FSM states
RUN     = 5
FIRE    = 6
# turret image FSM states
CAPTURE = 7
PARSE   = 8
# old state variables
IDLE    = 9
LOCATE  = 10
ROTATE  = 11
RESET   = 12

class turret_gen_class:
    '''!
    Class to help perform the turret process.
    '''
    def __init__(self, task_name):
        '''!
        Initializes an instance of the turret generator class. 
        
        This initialization creates instances of all objects necessary for the 
        turret process. Initializes the pin to read the start button on pin
        PC2. Initializes the builtin LED (LD2) to be an output. Initializes the 
        motor to rotate the turret on pins PC1, PA0, PA1, timer 5. Initializes 
        the encoder reader on pins PC6, PC7, timer 8. Initializes the proportional 
        controller with this motor driver and this encoder reader. Initializes 
        the mlx camera on I2C bus 1 which is pins PB8 and PB9. Initializes the 
        servo on pin PB3, timer 2 to have a 20ms period and in its initial 
        position.
        @param      task_name -> String representing the name of this task.
        @returns    None.
        '''
        # indicate initializing process
        print(f'Initializing objects for turret control... ', end='')
        # initializes task name
        self.task_name = task_name
        # intialize start process flag
        self.start_flag = 0
        # initialize pin for active low start button
        self.sw1 = pyb.Pin(pyb.Pin.board.PC2, pyb.Pin.IN)
        # initialize on board LED for start indication
        self.led = pyb.Pin(pyb.Pin.board.PA5, pyb.Pin.OUT_PP)
        self.led.low()
        # initialize motor driver for rotating turret
        self.motor = motor_driver.MotorDriver(
            pyb.Pin.board.PC1,
            pyb.Pin.board.PA0, 
            pyb.Pin.board.PA1,
            timer=5)
        self.motor.set_duty_cycle(0)
        # intialize encoder reader for rotating turret
        self.encoder = encoder_reader.Encoder(
            pyb.Pin.board.PC6,
            pyb.Pin.board.PC7,
            timer_num=8)
        self.encoder.zero()
        # initialize proportional controller
        self.pcontrol = proportional_controller.ProportionalController(
            Kp=0,   #1.0    #TODO
            Kd=0,   #0.2    #TODO
            setpoint=0,
            setvel=0,
            actuate=self.motor.set_duty_cycle,
            sense=self.encoder.read,
            data_points=MOTOR_CONTROL_POINTS)
        # initialize mlx camera
        self.camera = mlx_cam.MLX_Cam(i2c=I2C(1))
        self.camera._camera.refresh_rate = 10.0
        # initialize servo
        self.servo = servo_driver.ServoDriver(
            pin=pyb.Pin.board.PB3,
            timer_num=2,
            channel_num=2,
            period=19999,
            ps=79)
        # initialize servo to farthest position counter clockwise
        self.servo.set_pulse_width(SERVO_START_POS)
        # initialize image
        self.image = None
        # initialize loop value
        self.loop_val = 0
        # initialize setpoint
        self.setpoint = 0
        # initialize start time
        self.start_time = utime.ticks_ms()

        # indicate done initializing objects
        print(f'Done.')


    def get_start_button(self):
        '''!
        Returns the active low start button object.
        @param      None.
        @returns    Pin object to read the active low start button value from.
        '''
        return self.sw1

    def get_motor(self):
        '''!
        Returns the motor driver object associated with this turret.
        @param      None.
        @returns    The MotorDriver object used to rotate this turret.
        '''
        return self.motor

    def get_servo(self):
        '''!
        Returns the servo driver object associated with this turret.
        @param      None.
        @returns    The ServoDriver object used to fire this turret.
        '''
        return self.servo

    def get_led(self):
        '''!
        Returns the pin object used to actuate the builtin LED LD2.
        @param      None.
        @returns    Pin object to set the value of the builtin LED to.
        '''
        return self.led
    
    def turret_angle_to_encoder_val(self,angle):
        '''!
        Converts the input angle to an encoder value for the turret. The 
        reference for this input angle should be 180 degrees from the initial 
        direction of the turret. This function will take this into account, 
        and will add 180 degrees worth of encoder value to the angle input
        to this function. This allows, this encoder value to be used as the 
        setpoint for the proportional controller. Overall, whereas the output 
        is favorable for dealing with the turret, the input is favorable for 
        dealing with the thermal image. Assumes that clockwise is the
        positive direction for the input angle as well as the positive 
        direction for the returned encoder value.
        @param      angle -> Angle to convert to encoder value. Expects 
                    this value to be in units of degrees.
        @returns    The encoder value for the turret, representative of the 
                    input angle.
        '''
        return (angle + 180) * ENCODER_COUNT_PER_REV / 360

    def get_center_of_mass(self,data:list[int]):
        '''!
        Calculates the center of mass of the line of integer data. Uses the 
        indices of the data as the weights on the data. If the list is empty,
        returns 0
        @param      data -> A list of number data (data should be integers).
        @returns    The center of mass of the input line of data.
        '''
        # check for an empty list
        if not len(data):
            return 0
        
        # otherwise, data is present, calculate the center of mass
        weighted_mass = 0
        total_mass = 0
        for idx, val in enumerate(data):
            weighted_mass += idx * val
            total_mass += val
        
        # return the center of mass = weighted mass / total mass
        return weighted_mass / total_mass

    def center_of_mass_to_degrees(self,cm):
        '''!
        Converts the inputted center of mass to the direction of the 
        center of mass in units of degrees. The units of the input center of 
        mass should be columns of thermal image data. The reference of zero 
        for these columns is on the left side of the camera. For the returned
        angle to be accurate, the camera should be placed according to the 
        global constants at the top of this file, along the center of the 
        table. The reference for the returned angle is from the line down 
        the center of the table in the direction of the target. A positive 
        angle is in the clockwise direction.
        @param      cm -> Center of mass of thermal image data (in units of
                    columns of thermal image data).
        @returns    Angle representing the direction of the center of mass
                    of thermal image data. Angle returned in units of 
                    degrees.
        '''
        # reassign global variables for easier coding
        c = PERP_DIST_CAMERA_TO_TARGET
        t = PERP_DIST_TURRET_TO_TARGET
        camera_fov = CAMERA_FOV_ANGLE

        # calculate the degrees of the FOV from the perspective of the turret
        turret_fov = math.atan((c*math.tan(math.radians(camera_fov/2)))/t)
        turret_fov = 2 * math.degrees(turret_fov)
        
        # get the number of cols of thermal image data
        cols = mlx_cam.NUM_COLS

        # calculate the degrees of distance between columns
        dd = turret_fov / cols # units = [degrees / col]

        # convert center of mass to degrees
        #   the 0 of center of mass is referenced from the left-most side 
        #   of the FOV of the thermal camera
        cm_d = cm * dd # units = [cols] * [degrees/col] = [degrees]

        # change reference so that zero is perpendicular to the motion plane
        #   of the target. thats, perpendicular to the center direction of 
        #   turret fire.
        cm_d = cm_d - (turret_fov / 2)

        # return the angle representing the direction of the target
        return cm_d
        
    def parse_image(self,camera,image):
        '''!
        Uses the captured thermal image data to return the direction of the 
        target in units of encoder counts. First, parses image data to find 
        the row with the largest mean value. Then uses this row of data to 
        calculate the center of mass for this row. Then uses this center of 
        mass to find the direct the turret should be aimed and converts this
        value to units of encoder counts to be used with the proportional
        controller. Assumes camera to be pointed 180 degrees away from the 
        direction the turret is pointed. Positive rotational direction is 
        defined to be clockwise.
        @param      camera -> MLX_Cam object used to obtain image. 
        @param      image -> Thermal image from MLX_Cam object.
        @returns    Direction of target in units of encoder counts.
        '''
        # initialize list of image data
        data = []
        # initialize list of means of each row of image data
        mean = []
        # get each row of image data
        for line in camera.get_num_csv(image,limits=THERMAL_LIMITS):
            # append row to image data list
            data.append(line)
            # calculate mean of row and append to mean list
            mean.append(sum(line) / len(line))
        # find the index of the row with largest mean
        largest_mean_idx = mean.index(max(mean))
        # calculate the center of mass of the row with the largest mean
        cm = self.get_center_of_mass(data[largest_mean_idx])
        # find direction of center of mass
        dir = self.center_of_mass_to_degrees(cm)
        # convert this direction to units of encoder counts
        enc_dir = self.turret_angle_to_encoder_val(dir)
        # return the direction in units of encoder counts
        return enc_dir

    def turret_process_gen_fun(self):
        '''!
        The generator function used to run the finite state machine of the turret
        process as a task. Locates opponent, rotates turret towards opponent, and 
        fires the Nerf dart.
        @param      None.
        @returns    Yields the state to run next.
        '''
        # initialize state for
        state = IDLE
        # run the finite state machine
        while True:
            # yield the state about to run next
            yield state

            # check if is in IDLE state
            if state == IDLE:
                # check active low start button
                if not self.sw1.value():
                    # indicate button pressed
                    self.led.high()
                    # next state is WAIT
                    state = WAIT
                    print('Done with IDLE state.')
                    print('Starting turret process.')
                # otherwise, idle while waiting for start signal
                else:
                    # keep values low
                    self.motor.set_duty_cycle(0)
                    self.encoder.zero()
            
            # check if is in WAIT state
            elif state == WAIT:
                # check if completed wait duration
                if self.loop_val >= WAIT_TIME // MOTOR_CONTROL_INTERVAL:
                    # reset loop value
                    self.loop_val = 0
                    # next state is LOCATE
                    state = LOCATE
                    print('Done with WAIT state.')
                # otherwise, increment loop value
                else:
                    self.loop_val += 1

            # check if is in LOCATE state
            elif state == LOCATE:
                # check for full image
                if not self.image:
                    # check if completed image wait duration
                    if self.loop_val <= 0:
                        # check for image
                        self.image = self.camera.get_image_nonblocking()
                        # reset loop value
                        self.loop_val = IMAGE_WAIT_TIME // MOTOR_CONTROL_INTERVAL
                    # otherwise, continue with image wait duration
                    else:
                        # decrement loop value
                        self.loop_val -= 1
                
                # otherwise, have the image
                else:
                    # reset loop value
                    self.loop_val = 0
                    # next state is PARSE
                    state = PARSE
                    print('Done with LOCATE state.')

            # check if is in PARSE state
            elif state == PARSE:
                # parse image for target direction
                self.setpoint = self.parse_image(self.camera,self.image)
                # set the setpoint of the proportional controller
                self.pcontrol.set_setpoint(self.setpoint)
                # next state is ROTATE
                state = ROTATE
                print('Done with PARSE state.')
            
            # check if is in ROTATE state
            elif state == ROTATE:
                # actuate motor with proportional controller

                # check if is the first time in this state
                if self.loop_val == 0:
                    # initialize start time
                    self.start_time = utime.ticks_ms()
                
                # execute proportional controller desired number of times
                if self.loop_val <= MOTOR_CONTROL_POINTS:
                    # run the proportional controller
                    self.pcontrol.run(MOTOR_CONTROL_INTERVAL,self.start_time)
                    # increment the number of times 
                    self.loop_val += 1
                # otherwise, done with rotating
                else:
                    # reset the loop value
                    self.loop_val = 0
                    # next state is FIRE
                    state = FIRE
                    print('Done with ROTATE state.')

            # check if is in FIRE state
            elif state == FIRE:
                # check if still need to wait for servo to finish moving
                if self.loop_val <= SERVO_WAIT_TIME // MOTOR_CONTROL_INTERVAL:
                    # actuate servo to fire the turret
                    self.servo.set_pulse_width(SERVO_START_POS)
                    # increment loop_val
                    self.loop_val += 1
                # otherwise, done firing the turret
                else:
                    # next state is RESET
                    state = RESET
                    print('Done with FIRE state.')

            # check if is in RESET state
            elif state == RESET:
                # reset values
                self.motor.set_duty_cycle(0)
                self.servo.set_pulse_width(1.0)
                self.led.low()
                self.image = None
                self.loop_val = 0
                self.setpoint = 0
                self.pcontrol.set_setpoint(0)
                self.encoder.zero()

                # indicate done with turret process
                print('Done with RESET state.')
                print('Done with turret process.')
                
                # next state is IDLE
                state = IDLE

            # otherwise, incorrect state value
            else:
                raise ValueError('Invalid state')
        # end while
            
    def button_task_gen_fun(self):
        '''!
        This generator function funs the state machine for the button task.
        Constantly checks the state of the active low start button. After
        the button is pressed, initializes values for this to be the start
        position and waits 5 seconds for opponents to position themself.
        @param      None.
        @returns    None.
        '''
        # initialize state
        state = CHECK
        # initialize wait counter
        wait_counter = 0
        # run the FSM
        while True:
            # yield the state about to execute
            yield state
            # CHECK state
            if state == CHECK:
                # check if the active low button is pressed
                if not self.sw1.value():
                    # next state is INIT
                    state = INIT
            # INIT state
            elif state == INIT:
                # indicate process started with LED
                self.led.high()
                # intialize servo to farthest counter clockwise position
                self.servo.set_pulse_width(SERVO_START_POS)
                # initialize proportional controller setpoint to be in line with the camera
                #   in line with camera = 0 degrees from camera reference
                #   in line with camera = 180 degrees from initial turret direction
                self.setpoint = self.turret_angle_to_encoder_val(0)
                self.pcontrol.set_setpoint(self.setpoint)
                # initialize this turret position to be zero
                self.encoder.zero()
                # next state is WAIT
                state = WAIT
            # WAIT state
            elif state == WAIT:
                # check if done waiting
                if wait_counter >= WAIT_TIME // BUTTON_TASK_INTERVAL:
                    # set start flag
                    self.start_flag = 1
                    # reset wait_counter
                    wait_counter = 0
                    # next state is DONE
                    state = DONE
                # otherwise, still waiting
                else:
                    # increment wait counter
                    wait_counter += 1
            # DONE state
            elif state == DONE:
                # do nothing in this state bc done with button functionality
                pass
            # handle bad state values
            else:
                raise ValueError(f'Incorrect state for button task. Value was {state}.')
        # end while
        # TODO
        
    def image_task_gen_fun(self):
        '''!
        This generator function runs the state machine for the image task.
        Constantly tries to capture an image from the thermal camera. Once
        a valid image is read, the location of the target is derived and 
        stored in the setpoint attribute of this class for the turret task
        to use as an input to the closed-loop proportional controller.
        @param      None.
        @returns    Yields the state of the FSM.
        '''
        # initialize the state variable
        state = CAPTURE
        # initialize the wait counter 
        wait_counter = 0
        # run the FSM
        while True:
            # yield the state that will execute next
            yield state
            # CAPTURE state
            if state == CAPTURE:
                # check that the start flag has been set (i.e. done with initial wait)
                if self.start_flag:
                    # check for image
                    if not self.image:
                        # try to get the image
                        self.image = self.camera.get_image_nonblocking()
                    # otherwise, got the image
                    else:
                        # next state is PARSE
                        state = PARSE
            # PARSE state
            elif state == PARSE:
                # check if is first time in this state
                if wait_counter == 0:
                    # derive setpoint from image data
                    self.setpoint = self.parse_image(self.camera,self.image)
                    # increment the wait counter
                    wait_counter += 1
                # check if done waiting for motor to rotate, and is time to get another image
                elif wait_counter >= MOTOR_CONTROL_PERIOD // MOTOR_CONTROL_INTERVAL:
                    # reset wait counter
                    wait_counter = 0
                    # next state is CAPTURE
                    state = CAPTURE
                # otherwise, have already set the setpoint for this image
                else:
                    # increment wait counter
                    wait_counter += 1
                # next state is CAPTURE
                state = CAPTURE
            # handle bad values for state
            else:
                raise ValueError(f'Incorrect state for image task. Value was {state}.')
        # end while
        # TODO
    
    def rotate_task_gen_fun(self):
        '''!
        This generator function runs thet state machine for the rotate task.
        This task runs a closed loop proportional controller to aim the 
        turret. Then after a globally defined interval, will actuate the 
        servo and fire the Nerf dart.
        @param      None.
        @returns    Yields the state of the FSM.
        '''
        # initialize the state variable
        state = RUN
        # initialize wait counter
        wait_counter = 0
        # run the FSM
        while True:
            # yield the state that will execute next
            yield state
            # RUN state
            if state == RUN:
                # check that the start flag has been set
                if self.start_flag:
                    # check if it is the first time in this state
                    if wait_counter == 0:
                        # get the start time
                        self.start_time = utime.ticks_ms()
                        # increment the wait counter
                        wait_counter += 1
                    # check if done rotating the turret
                    if wait_counter >= MOTOR_CONTROL_PERIOD // MOTOR_CONTROL_INTERVAL:
                        # reset wait counter
                        wait_counter = 0
                        # next state is FIRE
                        state = FIRE
                    # set the setpoint in the proportional controller
                    self.pcontrol.set_setpoint(self.setpoint)
                    # run the proportional controller
                    self.pcontrol.run(MOTOR_CONTROL_INTERVAL,self.start_time)
                    # increment the wait counter
                    wait_counter += 1
            # FIRE state
            elif state == FIRE:
                # check if is first time in this state
                if wait_counter == 0:
                    # actuate servo to pull trigger
                    self.servo.set_pulse_width(SERVO_PULLED_POS)
                    # increment wait counter
                    wait_counter += 1
                # check if done waiting
                elif wait_counter >= SERVO_WAIT_TIME // MOTOR_CONTROL_INTERVAL:
                    # reset servo to initial position
                    self.servo.set_pulse_width(SERVO_START_POS)
                    # reset wait counter
                    wait_counter = 0
                    # next state is RUN
                    state = RUN
                    # turn off start flag
                    self.start_flag = 0
                # otherwise, wait for servo to move
                else:
                    # increment wait counter
                    wait_counter += 1
            # handle bad state values
            else:
                raise ValueError(f'Incorrect state value for rotate task. Value was {state}.')
        # end while
        # TODO

def main():
    '''!
    This function is only executed when this file is ran as the main file.
    Runs the turret process as a task. 
    @param      None.
    @returns    None.
    '''
    # initialize the turret object
    turret = turret_gen_class('Turret 1')
    # initialize the turret's button generator function
    button_gen_fun = turret.button_task_gen_fun
    # initialize the turret's rotate generator function
    rotate_gen_fun = turret.rotate_task_gen_fun
    # initialize the turret's image generator function
    image_gen_fun = turret.image_task_gen_fun
    # initialize the button task
    button_task = cotask.Task(button_gen_fun,
                              name="Button_Task",
                              priority=0,                       # TODO
                              period=BUTTON_TASK_INTERVAL,      # TODO
                              profile=True,
                              trace=False,
                              shares=None)
    # initialize the rotate task
    rotate_task = cotask.Task(rotate_gen_fun, 
                              name="Rotate_Task", 
                              priority=1,                       # TODO
                              period=MOTOR_CONTROL_INTERVAL,    # TODO
                              profile=True, 
                              trace=False, 
                              shares=None)
    # initialize the image task
    image_task = cotask.Task(image_gen_fun,
                             name="Image_Task",
                             priority=2,                # TODO
                             period=IMAGE_WAIT_TIME,    # TODO
                             profile=True,
                             trace=False,
                             shares=None)
    # add button task to task list
    cotask.task_list.append(button_task)
    # add rotate task to task list
    cotask.task_list.append(rotate_task)
    # add image task to task list
    cotask.task_list.append(image_task)

    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect()

    # attempt turret process
    try:
        # get the active low start button object
        button = turret.get_start_button()
        # continuously check if the button is pressed
        while not button.value():
            pass
        # button has been pressed, run tasks
        while True:
            cotask.task_list.pri_sched()

    # caught the Ctrl+C (^C) signal
    except KeyboardInterrupt:
        # turn off servo
        turret.get_servo().zero()
        # turn motor off
        turret.get_motor().set_duty_cycle(0)
        # turn builtin LED off
        turret.get_led().low()
        # indicate exitting main
        print(f'\nExitting main due to KeyboardInterrupt\n\n')


# This main code is run if this file is the main program but won't run if this
# file is imported as a module by some other main program
if __name__ == '__main__':

    main()
    

