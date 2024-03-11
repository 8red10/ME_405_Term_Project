'''!
@file encoder_reader.py
This file contains the Encoder class that can be used to read the position of a motor encoder 
and will handle underflows and overflows appropriately by continuing the count in the correct 
direction. When run as main, this file will initialize an enocder object and every 50 
milliseconds will print the value read from the encoder.

Motor wire connections:
    Blue	    Encoder channel B
    Yellow	    Encoder channel A
    Red	        Encoder 5V supply (should be connected to the 3.3V output)
    Black	    Encoder ground
    Orange	    Motor power
    Green	    Motor power
    (None)	    No connection

Possible encoder timer configurations:
    Timer 4
        Channel 1: pin PB6
        Channel 2: pin PB7
    Timer 8
        Channel 1: pin PC6
        Channel 2: pin PC7

@author Jack Krammer and Jason Chang
@date   13-Feb-2024
@copyright (c) 2024 by mecha04 and released under MIT License
'''

import pyb
import utime

class Encoder:
    '''!
    This class implements a motor encoder reader for the ME 405 kit.
    '''
    def __init__(self,ch1_pin,ch2_pin,timer_num):
        '''!
        Creates a motor encoder reader. The starting position of the motor is set as the zero 
        position. Assumes that the input pins are pin objects and the input timer number is the 
        integer number representing the timer to set up as the encoder reader. 
        @param      ch1_pin -> Pin object associated with channel 1 of the input timer number
        @param      ch2_pin -> Pin object associated with channel 2 of the input timer number 
        @param      timer_num -> Integer representing the timer number to use to read the encoder
        @returns    None.
        '''
        # initialize the pins
        self.ch1_pin = ch1_pin
        self.ch2_pin = ch2_pin
        # initialize the timer and associated channels
        self.timer = pyb.Timer(timer_num, prescaler=0, period=65535) # max period for timers 4 and 8
        self.ch1 = self.timer.channel(1, pyb.Timer.ENC_AB, pin=self.ch1_pin)
        self.ch2 = self.timer.channel(2, pyb.Timer.ENC_AB, pin=self.ch2_pin)
        # intialize the position variable to be zero
        self.position = 0
        # intialize previous count variables
        self.prev_count = 0

    def read(self):
        '''!
        First, calls the update_position function to check for overflows and underflows of the 
        timer position, and updates the integer position variable accordingly. Then returns the 
        current positon of the motor as represented by the timer count value.
        @param      None.
        @returns    The current position of the motor.
        '''
        # update the positon variable
        self.update_position()
        # return the updated position
        return self.position

    def zero(self):
        '''!
        Sets the count to zero at the current position. In effect, doesn't change the timer 
        count variable to ensure the deltas can continue to be calucluated, but resets the 
        position variable to zero.
        @param      None.
        @returns    None.
        '''
        # reset the position variable to be zero
        self.position = 0

    def update_position(self):
        '''!
        Updates the position variable based on the timer counter value. Checks for underflow 
        and overflow and adjusts the position variable accordingly. In effect, if there is an 
        overflow, the position variable will continue to count up, and if there is an underflow, 
        the position variable will continue to count down. 
        @param      None.
        @returns    None.
        '''
        # get the auto-reload value
        ar = self.timer.period()
        # get the current timer count
        count = self.timer.counter()
        # calculate the delta from the previous count
        delta = count - self.prev_count

        # check for underflow or overflow
        val = (ar + 1) / 2      # underflow comparison value
        neg_val = -1 * val      # overflow comparision value
        if delta > val:
            # adjust for underflow
            delta -= ar + 1
            # print(f'underflow happened!')
        elif delta < neg_val:
            # adjust for overflow
            delta += ar + 1
            # print(f'overflow happened!')

        # update the position variable with the timer delta
        self.position += delta
        # set the previous count
        self.prev_count = count


def main():
    '''!
    Initializes an enocder object on timer 8 with channel 1 on the associated pin PC6 
    and channel 2 on the associated pin PC7. Reads the value of the position every 50 
    milliseconds and prints the result. Handles keyboard interrupts smoothly. 
    @param      None.
    @returns    None.
    '''
    # run this code until a keyboard interrupt is forced
    try:
        # PB6 = timer 4, channel 1
        pc6 = pyb.Pin.board.PC6
        # PB7 = timer 4, channel 2
        pc7 = pyb.Pin.board.PC7
        # initialize encoder object
        encoder = Encoder(pc6, pc7, timer_num=8)

        # every 50 milliseconds read the value of the position
        while True:
            pos = encoder.read()
            print(f'the position is: {pos}')
            utime.sleep_ms(50)

    except KeyboardInterrupt:
        print(f'\nexiting main due to keyboard interrupt.\n\n')

# This main code is run if this file is the main program but won't run if this
# file is imported as a module by some other main program
if __name__ == '__main__':
    main()
