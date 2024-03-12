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

                            
@authors    Jack Krammer and Jason Chang
@date       12-Mar-2024
@copyright (c) 2024 by mecha04 and released under MIT License
'''

# imports here
import pyb
import utime
import motor_driver
import encoder_reader
import proportional_controller
import mlx_cam
import servo_driver
from array import array
import math
import cotask
import task_share
import gc

# global constants
ENCODER_COUNT_PER_REV   = 16384
MOTOR_CONTROL_INTERVAL  = 10        # milliseconds
MOTOR_CONTROL_PERIOD    = 2000      # milliseconds
MOTOR_CONTROL_POINTS    = MOTOR_CONTROL_PERIOD // MOTOR_CONTROL_INTERVAL
THERMAL_LIMITS          = (0,100)

# global wait time constants
WAIT_TIME               = 5000      # milliseconds
IMAGE_WAIT_TIME         = 50        # milliseconds
SERVO_WAIT_TIME         = 2000      # milliseconds

# global geometric placement variables
PERP_DIST_CAMERA_TO_TARGET      = 8     # feet
PERP_DIST_TURRET_TO_TARGET      = 14    # feet
CAMERA_FOV_ANGLE                = 55    # degrees

# global state variables
IDLE    = 1
WAIT    = 2
LOCATE  = 3
PARSE   = 4
ROTATE  = 5
FIRE    = 6
RESET   = 7


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
        the mlx camera on I2C bus 2 which is pins PB10 and PB11. Initializes the 
        servo on pin PB3, timer 2 to have a 20ms period and in its initial 
        position.
        @param      task_name -> String representing the name of this task.
        @returns    None.
        '''
        # indicate initializing process
        print(f'Initializing objects for turret control... ', end='')
        # initializes task name
        self.task_name = task_name
        # intializes state to IDLE
        self.state = IDLE
        # initialize pin for active low start button
        self.sw1 = pyb.Pin(pyb.Pin.board.PC2, pyb.Pin.IN)
        #initialize on board LED for start indication
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
            timer=8)
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
        self.camera = mlx_cam.MLX_Cam(i2c=pyb.I2C(2))
        self.camera._camera.refresh_rate = 10.0
        # initialize servo
        self.servo = servo_driver.ServoDriver(
            pin=pyb.Pin.board.PB3,
            timer_num=2,
            channel_num=2,
            period=19999,
            ps=79)
        # sets servo to farthest position clockwise
        self.servo.set_pulse_width(1.0)
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
        # run the finite state machine
        while True:
            # yield the state about to run next
            yield self.state

            # check if is in IDLE state
            if self.state == IDLE:
                # check active low start button
                if not self.sw1.value():
                    # indicate button pressed
                    self.led.high()
                    # next state is WAIT
                    self.state = WAIT
                    print('Done with IDLE state.')
                    print('Starting turret process.')
                # otherwise, idle while waiting for start signal
                else:
                    # keep values low
                    self.motor.set_duty_cycle(0)
                    self.encoder.zero()
            
            # check if is in WAIT state
            elif self.state == WAIT:
                # check if completed wait duration
                if self.loop_val >= WAIT_TIME // MOTOR_CONTROL_INTERVAL:
                    # reset loop value
                    self.loop_val = 0
                    # next state is LOCATE
                    self.state = LOCATE
                    print('Done with WAIT state.')
                # otherwise, increment loop value
                else:
                    self.loop_val += 1

            # check if is in LOCATE state
            elif self.state == LOCATE:
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
                    self.state = PARSE
                    print('Done with LOCATE state.')

            # check if is in PARSE state
            elif self.state == PARSE:
                # parse image for target direction
                self.setpoint = self.parse_image(self.camera,self.image)
                # set the setpoint of the proportional controller
                self.pcontrol.set_setpoint(self.setpoint)
                # next state is ROTATE
                self.state = ROTATE
                print('Done with PARSE state.')
            
            # check if is in ROTATE state
            elif self.state == ROTATE:
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
                    self.state = FIRE
                    print('Done with ROTATE state.')

            # check if is in FIRE state
            elif self.state == FIRE:
                # check if still need to wait for servo to finish moving
                if self.loop_val <= SERVO_WAIT_TIME // MOTOR_CONTROL_INTERVAL:
                    # actuate servo to fire the turret
                    self.servo.set_pulse_width(2.0)
                    # increment loop_val
                    self.loop_val += 1
                # otherwise, done firing the turret
                else:
                    # next state is RESET
                    self.state = RESET
                    print('Done with FIRE state.')

            # check if is in RESET state
            elif self.state == RESET:
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
                self.state = IDLE

            # otherwise, incorrect state value
            else:
                raise ValueError('Invalid state')
            
        # end while
        

def main():
    '''!
    This function is only executed when this file is ran as the main file.
    Runs the turret process as a task. 
    @param      None.
    @returns    None.
    '''
    # initialize the turret object
    turret = turret_gen_class('Turret 1')
    # initialize the turret process generator function
    turret_gen_fun = turret.turret_process_gen_fun()
    # initialize the turret task
    turret_task_1 = cotask.Task(turret_gen_fun, 
                                name="Turret_Task_1", 
                                priority=1, 
                                period=MOTOR_CONTROL_INTERVAL,
                                profile=True, 
                                trace=False, 
                                shares=None)
    # add turret task to task list
    cotask.task_list.append(turret_task_1)

    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect()

    # attempt turret process
    try:
        while True:
            cotask.task_list.pri_sched()

    except KeyboardInterrupt:
        # turn off servo
        turret.servo.zero()
        # turn motor off
        turret.motor.set_duty_cycle(0)
        # turn builtin LED off
        turret.led.low()
        # indicate exitting main
        print(f'\nExitting main due to KeyboardInterrupt\n\n')


def test():
    '''!
    This function is used to test the turret process.
    @param      None.
    @returns    None.
    '''

    turret = turret_gen_class('testing turret')

    

# This main code is run if this file is the main program but won't run if this
# file is imported as a module by some other main program
if __name__ == '__main__':
    
    # main()

    test()
    

