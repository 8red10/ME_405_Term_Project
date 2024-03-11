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
from array import array
import math


# global constants
ENCODER_COUNT_PER_REV   = 16384
MOTOR_CONTROL_INTERVAL  = 10        # milliseconds
MOTOR_CONTROL_PERIOD    = 2000      # milliseconds
MOTOR_CONTROL_POINTS    = MOTOR_CONTROL_PERIOD // MOTOR_CONTROL_INTERVAL
# THERMAL_THRESHOLD       = 0 #20
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


def turret_angle_to_encoder_val(angle):
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


def get_center_of_mass(data: list[int]):
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


def center_of_mass_to_degrees(cm):
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


def parse_image(camera,image):
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
    cm = get_center_of_mass(data[largest_mean_idx])
    # find direction of center of mass
    dir = center_of_mass_to_degrees(cm)
    # convert this direction to units of encoder counts
    enc_dir = turret_angle_to_encoder_val(dir)
    # return the direction in units of encoder counts
    return enc_dir


def init_objects():
    '''!
    Initializes the pins and drivers required for the turret. Returns these
    objects in a tuple. Initializes the pin to read the start button on pin
    PC2. Initializes the builtin LED (LD2) to be an output. Initializes the 
    motor to rotate the turret on pins PC1, PA0, PA1, timer 5. Initializes 
    the encoder reader on pins PC6, PC7, timer 8. Initializes the proportional 
    controller with this motor driver and this encoder reader. Initializes 
    the mlx camera on I2C bus 2 which is pins PB10 and PB11. Initializes 
    SERVO TODO --------------------------------------------------------------------------------------------------------
    -------------------------------------------------------------------------------------------------------------------
    -------------------------------------------------------------------------------------------------------------------
    -------------------------------------------------------------------------------------------------------------------
    -------------------------------------------------------------------------------------------------------------------
    -------------------------------------------------------------------------------------------------------------------
    -------------------------------------------------------------------------------------------------------------------
    -------------------------------------------------------------------------------------------------------------------
    -------------------------------------------------------------------------------------------------------------------
    @param      None.
    @returns    Tuple of objects initialized. Returns objects in order: 
                (start button, on board LED, motor driver, encoder reader, 
                proportional controller, mlx camera, servo).
    '''
    # indicate initializing process
    print(f'Initializing objects for turret control... ', end='')

    # initialize pin for active low start button
    sw1 = pyb.Pin(pyb.Pin.board.PC2, pyb.Pin.IN)
    #initialize on board LED for start indication
    ld2 = pyb.Pin(pyb.Pin.board.PA5, pyb.Pin.OUT_PP)
    ld2.low()

    # initialize motor driver for rotating turret
    motor = motor_driver.MotorDriver(
        pyb.Pin.board.PC1,
        pyb.Pin.board.PA0, 
        pyb.Pin.board.PA1,
        timer=5)
    motor.set_duty_cycle(0)

    # intialize encoder reader for rotating turret
    encoder = encoder_reader.Encoder(
        pyb.Pin.board.PC6,
        pyb.Pin.board.PC7,
        timer=8)
    encoder.zero()

    # initialize proportional controller
    pcontrol = proportional_controller.ProportionalController(
        Kp=0,   #1.0
        Kd=0,   #0.2
        setpoint=0,
        setvel=0,
        actuate=motor.set_duty_cycle,
        sense=encoder.read,
        data_points=MOTOR_CONTROL_POINTS)
    
    # initialize mlx camera
    camera = mlx_cam.MLX_Cam(i2c=pyb.I2C(2))
    camera._camera.refresh_rate = 10.0
    
    # initialize servo
    servo = None

    # indicate done initializing objects
    print(f'Done.')
    # returns tuple of objects initialized
    return (sw1, ld2, motor, encoder, pcontrol, camera, servo)


def turret_process_gen_fun():
    '''!
    The generator function used to run the finite state machine of the turret
    process as a task. Locates opponent, rotates turret towards opponent, and 
    fires the Nerf dart.
    @param      None.
    @returns    None.
    '''
    # intialize objects
    sw1, led, motor, encoder, pcontrol, camera, servo = init_objects()
    # initialize image
    image = None
    # initialize state to IDLE
    state = IDLE
    # initialize value for iterating sleeps and loops
    loop_val = 0
    # initialize setpoint direction of opponent
    setpoint = 0
    # initialize start time 
    start_time = utime.ticks_ms()

    # run the finite state machine
    while True:
        # yield the state about to run next
        yield state

        # check if is in IDLE state
        if state == IDLE:
            # check active low start button
            if not sw1.value():
                # indicate button pressed
                led.high()
                print(f'Starting turret process.')
                # next state is WAIT
                state = WAIT
            # otherwise, idle while waiting for start signal
            else:
                # keep values low
                motor.set_duty_cycle(0)
                encoder.zero()
        
        # check if is in WAIT state
        elif state == WAIT:
            # check if completed wait duration
            if loop_val >= WAIT_TIME // MOTOR_CONTROL_INTERVAL:
                # reset loop value
                loop_val = 0
                # next state is LOCATE
                state = LOCATE
            # otherwise, increment loop value
            else:
                loop_val += 1

        # check if is in LOCATE state
        elif state == LOCATE:
            # check for full image
            if not image:
                # check if completed image wait duration
                if loop_val <= 0:
                    # check for image
                    image = camera.get_image_nonblocking()
                    # reset loop value
                    loop_val = IMAGE_WAIT_TIME // MOTOR_CONTROL_INTERVAL
                # otherwise, continue with image wait duration
                else:
                    # decrement loop value
                    loop_val -= 1
            
            # otherwise, have the image
            else:
                # reset loop value
                loop_val = 0
                # next state is PARSE
                state = PARSE

        # check if is in PARSE state
        elif state == PARSE:
            # parse image for target direction
            setpoint = parse_image(camera,image)
            # set the setpoint of the proportional controller
            pcontrol.set_setpoint(setpoint)
            # next state is ROTATE
            state = ROTATE
        
        # check if is in ROTATE state
        elif state == ROTATE:
            # actuate motor with proportional controller

            # check if is the first time in this state
            if loop_val == 0:
                # initialize start time
                start_time = utime.ticks_ms()
            
            # execute proportional controller desired number of times
            if loop_val <= MOTOR_CONTROL_POINTS:
                # run the proportional controller
                pcontrol.run(MOTOR_CONTROL_INTERVAL,start_time)
                # increment the number of times 
                loop_val += 1
            # otherwise, done with rotating
            else:
                # reset the loop value
                loop_val = 0
                # next state is FIRE
                state = FIRE

        # check if is in FIRE state
        elif state == FIRE:
            # check if still need to wait for servo to finish moving
            if loop_val <= SERVO_WAIT_TIME // MOTOR_CONTROL_INTERVAL:
                # actuate servo to set level
                servo = None
                # increment loop_val
                loop_val += 1
            # otherwise, done firing the turret
            else:
                # next state is RESET
                state = RESET

        # check if is in RESET state
        elif state == RESET:
            # reset values
            motor.set_duty_cycle(0)
            servo = None
            led.low()
            image = None
            loop_val = 0
            setpoint = 0
            pcontrol.set_setpoint(0)
            encoder.zero()

            # indicate done with turret process
            print(f'Done with turret process.')
            
            # next state is IDLE
            state = IDLE

        # otherwise, incorrect state value
        else:
            raise ValueError('Invalid state')
        
    # end while
        

def main():
    '''!
    This function is only executed when this file is ran as the main file.
    @param      None.
    @returns    None.
    '''
    # intialize objects
    sw1, led, motor, encoder, pcontrol, camera, servo = init_objects()

    # attempt turret process
    try:
        # poll for active low start button
        while not sw1.value(): # keeps on checking until button press detected
            pass
        # illuminates on board LED once button press detected
        led.high()
        # inidicate the start of the process
        print(f'Turret process started.')

        # wait 5 seconds for opponent to position themselves
        utime.sleep_ms(5000)

        # capture thermal image ==> is blocking
        image = camera.get_image()

        # parse thermal image for location of opponent
        setpoint = parse_image(camera, image)
        pcontrol.set_setpoint(setpoint)

        # rotate turret to desired location
        start_time = utime.ticks_ms()
        for i in range(MOTOR_CONTROL_POINTS):
            pcontrol.run(MOTOR_CONTROL_INTERVAL,start_time)
            utime.sleep_ms(MOTOR_CONTROL_INTERVAL)
        
        # actuate servo to fire turret
        

        # reset values
        # turn off turret
        motor.set_duty_cycle(0)
        # turn off servo
        #servo.zero()
        # turn off led
        led.low()
        
        # indicate done with process
        print(f'Turning off turret.')


    except KeyboardInterrupt:
        # set motor to zero
        motor.set_duty_cycle(0)
        # set servo to zero (opposite of trigger actuation)
        #servo.zero()
        # turn builtin LED off
        led.low()
        # indicate exitting main
        print(f'\nExitting main due to KeyboardInterrupt\n\n')


# This main code is run if this file is the main program but won't run if this
# file is imported as a module by some other main program
if __name__ == '__main__':
    main()

