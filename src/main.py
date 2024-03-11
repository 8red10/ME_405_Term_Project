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

# global state variables
IDLE    = 1
WAIT    = 2
LOCATE  = 3
PARSE   = 4
ROTATE  = 5
FIRE    = 6
RESET   = 7



def rotate_gen_fun():
    '''!
    This is a generator function that runs the motor
    yields 1 while still moving to desired position
    then yields 0 when done
    so can do 

        @code
        while next(rotate_gen_fun)
            utime.sleep_ms(interval)
        @endcode
    '''
    pass

def init_objects():
    '''!
    Initializes the pins and drivers required for the turret. Returns these
    objects in a tuple. Initializes the pin to read the start button on pin
    PB6. Initializes the builtin LED to be an output. Initializes the motor 
    to rotate the turret on pins PC1, PA0, PA1, timer 5. Initializes the 
    encoder reader on pins PC6, PC7, timer 8. Initializes the proportional 
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
                proportional controller, mlx camera).
    '''
    # initialize pin for active low start button
    sw1 = pyb.Pin(pyb.Pin.board.PB6, pyb.Pin.IN)
    #initialize on board LED for start indication
    board_LED = pyb.Pin(pyb.Pin.board.PA5, pyb.Pin.OUT_PP)
    # initialize motor driver for rotating turret
    motor = motor_driver.MotorDriver(
        pyb.Pin.board.PC1,
        pyb.Pin.board.PA0, 
        pyb.Pin.board.PA1,
        timer=5
    )
    # intialize encoder reader for rotating turret
    encoder = encoder_reader.Encoder(
        pyb.Pin.board.PC6,
        pyb.Pin.board.PC7,
        timer=8
    )
    # initialize proportional controller
    pcontrol = proportional_controller.ProportionalController(
        Kp=0,   #1.0
        Kd=0,   #0.2
        setpoint=0,
        setvel=0,
        actuate=motor.set_duty_cycle,
        sense=encoder.read,
        data_points=1
    )
    # initialize mlx camera
    camera = mlx_cam.MLX_Cam(i2c=pyb.I2C(2))
    camera._camera.refresh_rate = 10.0

    # returns tuple of objects initialized
    return (sw1, motor, encoder, pcontrol, camera)


def parse_image(camera,image):
    '''!
    Uses camera object to get CSV version of image. Parses data to find
    @param      camera -> MLX_Cam object used to obtain image. 
    @param      image -> Thermal image from MLX_Cam object.
    @returns    
    '''
    pass


def main():
    '''!
    This function is only executed when this file is ran as the main file.
    Locates opponent, rotates turret towards opponent, and fires Nerf dart.
    @param      None.
    @returns    None.
    '''
    # indicate initializing process
    print(f'Initializing objects for turret control... ', end='')
    # intialize objects
    sw1, motor, encoder, pcontrol, camera = init_objects()


    # attempt turret process
    try:
        # poll for active low start button
        while not sw1.value(): # keeps on checking until button press detected
            pass
        # illumninates on board LED once button press detected


        # wait 5 seconds for opponent to position themselves
        utime.sleep_ms(5000)

        # capture thermal image
        # parse thermal image for location of opponent
        # rotate turret to desired location
        # actuate servo to fire turret
        # reset values

        
        # indicate done with process
        print(f'Turning off turret.')


    except KeyboardInterrupt:
        # set motor to zero
        motor.set_duty_cycle(0)
        # set servo to zero (opposite of trigger actuation)
        # servo.zero()
        # indicate exitting main
        print(f'\nExitting main due to KeyboardInterrupt\n\n')


# This main code is run if this file is the main program but won't run if this
# file is imported as a module by some other main program
if __name__ == '__main__':
    main()
