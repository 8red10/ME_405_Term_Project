'''!
@file test.py

A file to help test other files and other code.

@author     Jack Krammer
@date       12-Mar-2024
@copyright (c) 2024 by mecha04 and released under MIT License
'''

import pyb 
from machine import I2C
import utime
import mlx_cam
import motor_driver
import encoder_reader
import proportional_controller
import cotask
import task_share
import gc
import math

# globals
MOTOR_CONTROL_INTERVAL  = 20        # milliseconds
MOTOR_CONTROL_PERIOD    = 2000      # milliseconds
MOTOR_CONTROL_POINTS    = MOTOR_CONTROL_PERIOD // MOTOR_CONTROL_INTERVAL
ENCODER_COUNT_PER_REV   = 98218

THERMAL_LIMITS          = (0,100)

# global geometric placement variables
PERP_DIST_CAMERA_TO_TARGET      = 9     # feet
PERP_DIST_TURRET_TO_TARGET      = 17    # feet
CAMERA_FOV_ANGLE                = 55    # degrees

class test_gen_fun():
    '''!
    Class for creating generator functions to use with test task schedulers.
    '''
    def __init__(self,task_name):
        '''!
        Initializes an instance of the test_gen_fun class. This initializes
        a motor, encoder reader, and proportional controller to test various
        aspects of the ME 405 term project turret.
        '''
        # initialize task name
        self.task_name = task_name
        # initialize motor 
        self.motor = motor_driver.MotorDriver(
            pyb.Pin.board.PC1,
            pyb.Pin.board.PA0, 
            pyb.Pin.board.PA1,
            timer=5)
        self.motor.set_duty_cycle(0)
        # initialize encoder
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
        
        # TODO delete below b/c already initialized above
        # # intialize proportional controller parameters
        # self.pcontrol.set_Kp(1)
        # self.pcontrol.set_Kd(0.2)
        # self.pcontrol.set_setpoint(98024) # should be about 1 rev
        # self.pcontrol.set_setvel(0)
        # self.start_time = utime.ticks_ms()

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
        # return index pf max of row with largest mean
        return data.index(max(data))
    
        # TODO PREVIOUS METHOD BELOW
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
    
    def test_camera_data(self):
        '''!
        This function tests the camera and helper functions ability 
        to get the correct position.
        @param      None.
        @returns    None.
        '''
        # initialize the camera object
        self.camera = mlx_cam.MLX_Cam(i2c=I2C(1))
        # set and report the refresh rate
        print(f"Current refresh rate: {self.camera._camera.refresh_rate}")
        self.camera._camera.refresh_rate = 10.0
        print(f"Refresh rate is now:  {self.camera._camera.refresh_rate}")
        # initialize the image
        self.image = None
        # initialize the comparison time
        start_time = utime.ticks_ms()

        # repeatedly try to get image
        while not self.image:
            self.image = self.camera.get_image_nonblocking() # doing this once takes about 157ms, eventually done around 340ms
            print(f'time to check image = {utime.ticks_diff(utime.ticks_ms(),start_time)} ms')

        # try parsing image data
        self.setpoint = self.parse_image(self.camera,self.image)

    def pcontrol_gen_fun(self):
        '''!
        This is the generator function for the task of testing the proportional
        controller functions.
        @param      None.
        @returns    None.
        '''
        # intialize proportional controller parameters
        self.pcontrol.set_Kp(0.058)
        self.pcontrol.set_Kd(0.001)
        self.pcontrol.set_setpoint(9830)#(98024) # should be about 1 rev
        self.pcontrol.set_setvel(0)
        self.start_time = utime.ticks_ms()

        # set setpoint based on image data
        stpt = test_camera_data_no_class()
        self.pcontrol.set_setpoint(stpt)

        # store the kp and kd value 
        kp = self.pcontrol.Kp
        kd = self.pcontrol.Kd
        # store setpoint
        spt = self.pcontrol.setpoint
        svl = self.pcontrol.setvel

        # initialize previous encoder read value
        prev_val = 0

        # run the task
        while True:
            # run proportional encoder
            pwm = self.pcontrol.run(MOTOR_CONTROL_INTERVAL,self.start_time)
            # get encoder read
            val = self.encoder.read()
            # get encoder speed = (pos - prev_pos) * 1000 // interval 
            speed = (val - prev_val) * 1000 // MOTOR_CONTROL_INTERVAL
            # print encoder value and motor actuation value
            print(f'Encoder reads {val},\tread comp= {kp*(spt-val)},\t',end='')
            print(f'encoder vel = {speed},\tvel comp = {kd*(svl-speed)},  \tmotor value is {pwm}')
            # store previous encoder value
            prev_val = val
            # print the time if done
            if speed == 0:
                print(f'time since start = {utime.ticks_diff(utime.ticks_ms(),self.start_time)}',end='')
                print(f'\tdesired setpoint = {stpt}')
            # run one iteration of this loop per task interval
            yield 0

def test_new_pcontrol():
    '''!
    This function tests the code in the proportional_controller.py file.
    Specifically the run() function that now involves the derivative
    gain.
    @param      None.
    @returns    None.
    '''
    # intialize a test_gen_fun object
    turret = test_gen_fun('Test Turret')
    # intialize instance of pcontrol generator function
    pcontrol_gen_func = turret.pcontrol_gen_fun
    # initialize pcontrol task
    pcontrol_task = cotask.Task(pcontrol_gen_func,
                              name="P_Control_Task",
                              priority=1,                           
                              period=MOTOR_CONTROL_INTERVAL,        
                              profile=True,
                              trace=False,
                              shares=None)
    # add pcontrol task to list
    cotask.task_list.append(pcontrol_task)

    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect()

    # initialize start button
    sw1 = pyb.Pin(pyb.Pin.board.PC2, pyb.Pin.IN)
    # wait for active low button press before starting
    while sw1.value():
        utime.sleep_ms(100)

    # run the test
    try:
        while True:
            cotask.task_list.pri_sched()
    except KeyboardInterrupt:
        # turn off motor
        turret.motor.set_duty_cycle(0)
        # indicate exitting task scheduler
        print('\nKeyboardInterrupt detected. Exitting task scheduler. Turned motor off.\n')

    # indicate done with pcontrol test
    print('Done with new proportional controller tests.')

def test_encoder_setup():
    '''!
    This function tests the encoder setup with the term project turret.
    Specifically the direction of positive encoder ticks, the number
    of encoder ticks per revolution, orientation of motor wires for 
    the positive actuation values being in the clockwise direction.
    @param      None.
    @returns    None.
    '''
    # initialize motor 
    motor = motor_driver.MotorDriver(
        pyb.Pin.board.PC1,
        pyb.Pin.board.PA0, 
        pyb.Pin.board.PA1,
        timer=5)
    motor.set_duty_cycle(0)
    # initialize encoder
    encoder = encoder_reader.Encoder(
        pyb.Pin.board.PC6,
        pyb.Pin.board.PC7,
        timer_num=8)
    encoder.zero()
    
    # test which direction is positive
    #       RESULT = ccw = negative; cw = positive
    #       ^ when yellow (channel A) connected to PC6 and blue (channel B) connected to PC7
    try:
        while True:
            # print the current encoder value
            print(f'Encoder read value = {encoder.read()}')
            # sleep 50 ms
            utime.sleep_ms(50)
    except KeyboardInterrupt:
        print('\nKeyboardInterrupt detected. Done testing encoder values.\n')

    # test actual encoder count per rev
        # RESULT = around 96,000 --> find better value = 98,024.7
    # test motor direction according to wires
        # RESULT = orange wire should be on the B+ and green wire should be on B-
    # test which motor (A or B) of the H bridge
        # RESULT = motor for pins PC1, PA0, PA1 is labeled B on the H bridge
    # try:
    #     # turn motor on
    #     print('Turning motor on.\n')
    #     motor.set_duty_cycle(100)
    #     # print the encoder value
    #     while True:
    #         print(f'Encoder read value = {encoder.read()}')
    #         utime.sleep_ms(50)
    # except KeyboardInterrupt:
    #     # turn off motor
    #     motor.set_duty_cycle(0)
    #     print('KeyboardInterrupt detected. Turning motor off.\n')

        
    # indicate done with encoder tests
    print('Done with encoder setup tests.')

def test_camera_nonblock():
    '''!
    This function tests the code in the mlx_cam.py file.
    @param      None.
    @returns    None.
    '''
    # initialize a camera object
    camera = mlx_cam.MLX_Cam(i2c=I2C(1))
    # set and report the refresh rate
    print(f"Current refresh rate: {camera._camera.refresh_rate}")
    camera._camera.refresh_rate = 10.0
    print(f"Refresh rate is now:  {camera._camera.refresh_rate}")
    # initialize the image 
    image = None
    # initialize the comparison time
    start_time = utime.ticks_ms()
    # repeatedly try to get image
    while not image:
        image = camera.get_image_nonblocking() # doing this once takes about 157ms, eventually done around 340ms
        print(f'time to check image = {utime.ticks_diff(utime.ticks_ms(),start_time)} ms')
    # indicate done with this test
    print('Done testing image nonblock time.')

def test_camera_data_no_class():
    '''!
    This function tests the camera and helper functions ability 
    to get the correct position.
    @param      None.
    @returns    The setpoint derived from the thermal image.
    '''
    # initialize the camera object
    camera = mlx_cam.MLX_Cam(i2c=I2C(1))
    # set and report the refresh rate
    print(f"Current refresh rate: {camera._camera.refresh_rate}")
    camera._camera.refresh_rate = 10.0
    print(f"Refresh rate is now:  {camera._camera.refresh_rate}")
    # initialize the image
    image = None
    # initialize the comparison time
    start_time = utime.ticks_ms()

    # repeatedly try to get image
    while not image:
        image = camera.get_image_nonblocking() # doing this once takes about 157ms, eventually done around 340ms
        print(f'time to check image = {utime.ticks_diff(utime.ticks_ms(),start_time)} ms')

    # try parsing image data
    turret = test_gen_fun('Turret Test')
    setpoint = turret.parse_image(camera,image)
    print(f'\nThe setpoint found was \t{setpoint}\n')

    # print the CSV version of image
    print('About to print CSV data:')
    for line in camera.get_num_csv(image,limits=THERMAL_LIMITS):
        print(line)
        # print(len(line))
    
    # print the ASCII art version of image
    print('\nAbout to print ASCII art:')
    camera.ascii_art(image)

    # indicate done with the camera data test
    print('\nDone with camera parse data test.')

    # returns the setpoint found
    return setpoint

def start_button_test():
    '''!
    This function tests the input read from the start button.
    @param      None.
    @returns    None.
    '''
    # intialize input pin
    sw1 = pyb.Pin(pyb.Pin.board.PC2, pyb.Pin.IN)

    # continuously read pin input
    try:
        while True:
            print(f'Value on sw1 (PC2) is: {sw1.value()}')
            utime.sleep_ms(100)
    except KeyboardInterrupt:
        print('\nKeyboardInterrupt detected. Exitting start button test.\n')

def main():
    '''!
    This function only runs when this file is ran as the main file.
    @param      None.
    @returns    None.
    '''
    # test_camera_nonblock()
    # test_encoder_setup()
    test_new_pcontrol()
    # start_button_test()
    # test_camera_data_no_class()

    # indicate done with main
    print('Done with main of test.py\n')

if __name__ == '__main__':
    main()
