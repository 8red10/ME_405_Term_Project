'''!
@file servo_driver.py
@brief      This file contains code to drive a 1501 MG servo.
@details    Initializes a PWM timer to control the position of the 
            servo. Only needs one pin to output the PWM wave on. 
            Assumes the system clock to be 80MHz.

            Possible pin is PB3 with timer 2 channel 2.

            The 1501 MG servo has these characteristics: neutral 
            position corresponds with 1.5ms pulse width; pulse width
            range is [1.0, 2.0] milliseconds; rotating direction 
            counter clockwise when pulse width in range [1.5, 2.0]
            milliseconds. The red wire should be connected to a 5V
            source. The black wire should be connected to ground.
            The white wire is the signal wire that the PWM should be 
            on.

            Overall, for this servo, 1.5ms pulse width is the neutral 
            positon. To move the servo arm clockwise, a smaller pulse
            width should be used. And, to move the servo arm counter
            clockwise, a larger pulse width should be used. These 
            values for pulse width should be within the range [0.8, 
            2.2] milliseconds for the maximum reach and within the 
            range [1.0, 2.0] milliseconds for robust range.
            
            Equation for period: 
            period [ms] = (AR + 1)(PS + 1) * 1000 / fsysclk 

            Example finding desired period
            period [ms] = 1000 / ftimer
            1/ftimer = (AR + 1)(PS + 1) / fsysclk
            ftimer = 50Hz, AR = 19999, PS = 79

            When this file is run as a main file, a test function
            instantiates the servo on PB3 with timer 2 channel 2 
            and tests the behavior by setting the pulse width to 
            1.0 milliseconds, which should be a position 45 degrees
            clockwise. 

@author Jack Krammer and Jason Chang
@date   12-Mar-2024
@copyright (c) 2024 by mecha04 and released under MIT License
'''

import pyb 
import utime

F_SYSCLK = 80*1e6 # 80MHz

class ServoDriver:
    '''!
    This class implements a servo driver for the ME 405 term
    project.
    '''
    def __init__(self,pin=pyb.Pin.board.PB3,timer_num=2,channel_num=2,period=19999,ps=79):
        '''!
        Initializes a servo driver to output a PWM wave to the pin
        object input to this constructor, using the timer and 
        channel numbers also input to this constructor. Initializes
        the servo to its neutral position (1.5ms pulse width).
        @param      pin -> Pin object to have the PWM wave output
                    used to control the position of the servo. 
                    This Pin object should correspond with the 
                    timer and channel number input to this 
                    constructor. Defaults to a PB3 pyb.Pin object.
        @param      timer_num -> An integer representing the timer
                    to use to control the servo position. Defaults
                    to 2.
        @param      channel_num -> An integer representing the 
                    channel number of the timer that is used to 
                    control the servo position. Defaults to 2.
        @param      period -> An integer representing the period
                    to initialize the timer with. This is the value
                    loaded into the timer's AR register. Default 
                    value is 19999 to help create an actual frequency
                    of 50Hz corresponding to an actual period of 20ms.
        @param      ps -> An integer representinig the value to set 
                    the prescalar to. Default value is 79 to help 
                    create an actual frequency of 50Hz corresponding
                    to an actual period of 20ms.
        @returns    None.
        '''
        # initialize the pin
        self.pin = pyb.Pin(pin,pyb.Pin.OUT_PP)
        # initialize the AR register value of the timer
        self.AR = period
        # initialize the prescalar register value of the timer
        self.PS = ps 
        # calculate the actual period of the timer, in milliseconds
        self.actual_period = (self.AR + 1) * (self.PS + 1) * 1000 / F_SYSCLK
        # initialize the timer
        self.timer = pyb.Timer(timer_num,prescaler=ps,period=period)
        # initialize the channel to use for PWM
        self.channel = self.timer.channel(channel_num,pyb.Timer.PWM,pin=self.pin)
        # put the servo to its neutral position
        self.set_pulse_width(0)

    def get_whole_period(self):
        '''!
        Returns the actual period of the timer, with the prescalar and AR 
        values taken into account.
        @param      None.
        @returns    Acutal period of the timer in units of milliseconds.
        '''
        return self.actual_period

    def set_pulse_width(self,value):
        '''!
        Sets the pulse width of the servo to the input value. This 
        function is the most robust and most helpful for servo control.
        @param      value -> A number representing the number of 
                    milliseconds for the pulse width to be set to.
        @returns    None.
        '''
        # convert the value to units of AR register counts
        ar_counts = int(((value / 1000) * F_SYSCLK / (self.PS + 1)) - 1)
        # fit the value to the accepted range
        val = max(0,min(ar_counts,self.AR))
        # set the period of the servo
        self.channel.pulse_width(val)

    def zero(self):
        '''!
        Turns off any PWM input to the servo by setting the pulse width 
        value to zero. 
        @param      None.
        @returns    None.
        '''
        self.channel.pulse_width(0)

    def set_position(self,position:int):
        '''!
        Sets the position of the servo based on input.
        @param      position -> An integer in the range [0,100], where
                    0 is one extreme of its position and 100 is the other
                    extreme of its position.
        @returns    None.
        '''
        # fit the value to the range
        val = max(0,min(position,100))
        # set the position
        self.channel.pulse_width_percent(val)

    def set_duty_cycle(self,level:int):
        '''!
        Sets the duty cycle of the PWM to the input value.
        @param      level -> An integer in the range [0,100] representing
                    the duty cycle to set the servo to.
        @returns    None.
        '''
        # fit the value to the range
        val = max(0,min(100,level))
        # set the duty cycle
        self.channel.pulse_width_percent(val)


def test_servo():
    '''!
    This function tests this servo driver. Sets the pulse width of 
    the servo to be 1.5 milliseconds. Then waits for a KeyboardInterrupt
    before turning the servo off (setting pulse width to zero) and 
    exitting.
    @param      None.
    @returns    None.
    '''
    # indicate starting to test servo
    print('Starting to test servo.')
    # initialize servo
    servo = ServoDriver(pin=pyb.Pin.board.PB3,
                        timer_num=2,
                        channel_num=2,
                        period=19999,
                        ps=79)
    
    # test servo
    # pull the trigger 
    print('Pulling trigger.')
    servo.set_pulse_width(1.65)
    # wait 2 seconds
    print('Waiting 2 seconds.')
    utime.sleep_ms(2000)
    # initial position = trigger not pressed
    print('Reseting servo position. Waiting for KeyboardInterrupt...')
    servo.set_pulse_width(2.0)

    # wait for KeyboardInterrupt before cleaning up and exitting
    try:
        while True:
            pass
    except KeyboardInterrupt:
        # turn servo off
        servo.set_pulse_width(0)
        # indicate exitting
        print('\nExitting main: KeyboardInterrupt.\n\n')

    # indicate done testing servo
    print('Done testing servo.\n')
    

# This main code is run if this file is the main program but won't run if this
# file is imported as a module by some other main program
if __name__ == '__main__':
    test_servo()