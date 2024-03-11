'''!
@file motor_driver.py
This file contains code for the MotorDriver class that implements a motor driver for
the ME 405 kit. 

@author Jack Krammer and Jason Chang
@date   6-Feb-2024
@copyright (c) 2024 by mecha04 and released under MIT License
'''

import pyb
import utime

class MotorDriver:
    '''!
    This class implements a motor driver for the ME 405 kit.
    '''
    def __init__(self, en_pin, in1pin, in2pin, timer):
        '''!
        Creates a motor driver by initializing GPIO pins and turning off the motor for safety. 
        The pins inputted should just be the pin identifiers, they will be initialized here to the 
        specifications required. The timer input should just be the integer number of the timer to
        use for the two PWM channels. The timer will be set up such that channel 1 will control 
        one direction and channel 2 will control the other direction.
        @param en_pin ->    Enable pin. Will initialize to be open-drain output with pull-up resistor.
        @param in1pin ->    First input pin. Will initialize to be regular push-pull outputs.
        @param in2pin ->    Second input pin. Will initialize to be regular push-pull outputs.
        @param timer  ->    Timer object for PWM. Should just be an integer representing the timer number.
        '''
        # indicate the initialization
        # print('Creating motor driver...', end='')
        # set up the pins
        self.en_pin = pyb.Pin(en_pin, pyb.Pin.OUT_OD, pyb.Pin.PULL_UP)
        self.in1pin = pyb.Pin(in1pin, pyb.Pin.OUT_PP)
        self.in2pin = pyb.Pin(in2pin, pyb.Pin.OUT_PP)
        self.timer = pyb.Timer(timer, freq=20000) # 20kHz so humans can't hear the PWM
        # sets up timer channels
        self.ch1 = self.timer.channel(1, pyb.Timer.PWM, pin=self.in1pin)
        self.ch2 = self.timer.channel(2, pyb.Timer.PWM, pin=self.in2pin)
        # enables the motor
        self.en_pin.high()
        # initializes duty cycle to be zero to turn motor off for safety
        self.ch1.pulse_width_percent(0)
        self.ch2.pulse_width_percent(0)
        # indicate done creating the motor
        # print(' done.')
    
    def set_duty_cycle (self, level):
        '''!
        This method sets the duty cycle to be sent to the motor to the given level. Positive values
        cause torque in one direction, negative values in the opposite direction. The range for input
        values is [-100,100] inclusive. If the input values are outside of the range, the value is 
        clipped to be within range. i.e. anything < -100 is set to -100 and anything > 100 is set to 100.
        @param      level -> A signed integer holding the duty cycle of the voltage sent to the motor.
        @returns    None.
        '''
        # clip the level if necessary 
        if level > 100:
            level = 100
        elif level < -100:
            level = -100
        # indicate the level the motor is set to
        # print(f'Setting duty cycle to {level}%.')
        # turn on channel 1 for positive
        if level >= 0:
            self.ch2.pulse_width_percent(0)
            self.ch1.pulse_width_percent(level)
            # print('setting positive level')
        # turn on channel 2 for negative
        elif level < 0:
            self.ch1.pulse_width_percent(0)
            self.ch2.pulse_width_percent(abs(level))
            # print('setting negative level')

def main():
    '''!
    Sets up the pins required to use the MotorDriver class with a L6206 H-bridge and an 
    Arduino shield. Then spins the motor one way at 50% torque for 2 seconds, stops for
    2 seconds, then spins the motor the other way at 50% torque for 2 seconds, then 
    stops and disables the motor.This main code is run if this file is the main program 
    but won't run if this file is imported as a module by some other program.
    @param      None.
    @returns    None.
    '''
    # create a motor driver object
    motor  = MotorDriver(pyb.Pin.board.PA10, pyb.Pin.board.PB4, pyb.Pin.board.PB5, timer=3)
    
    # spin motor
    motor.set_duty_cycle(50)
    utime.sleep(2)
    motor.set_duty_cycle(0)
    utime.sleep(2)
    motor.set_duty_cycle(-50)
    utime.sleep(2)

    # turn motor torque off
    motor.set_duty_cycle(0)

# This main code is run if this file is the main program but won't run if this
# file is imported as a module by some other main program
if __name__ == '__main__':
    main()
    
    
    
    