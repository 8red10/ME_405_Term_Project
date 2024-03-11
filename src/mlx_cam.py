'''!
@file mlx_cam.py
@brief      A stripped down MLX90640 driver that produces only raw data, not 
            calibrated data, to use as an targetting system for the ME 405
            nerf turret duel.
@details    This file contains a wrapper that fascilitates the use of a Melexis
            MLX90640 thermal infrared camera for general use. The wrapper 
            contains a class MLX_Cam whose use is greatly simplified in
            comparison to that of the base class, MLX90640, by mwerezak. To 
            use this code, upload the directory @c mlx90640 from mwerezak with 
            all its contentts to the root directory of your MicroPython 
            device.
            When this file is run as the main file, the MLX90640 image detection
            feature is tested and print to output.
            The code in this file was originally authored by mwerezak, then 
            adapted by JR Ridgely, then editted for the purposes of the 
            mecha04 ME 405 lab group.

@author mwerezak Original files, Summer 2022
@author JR Ridgely Added simplified wrapper class @c MLX_Cam, January 2023
@author Jack Krammer and Jason Chang
@date   12-Mar-2024
@copyright (c) 2022-2024 by the authors and released under the GNU Public
    License, version 3.        
'''

import utime as time
from machine import Pin, I2C
from mlx90640 import MLX90640
from mlx90640.calibration import NUM_ROWS, NUM_COLS, IMAGE_SIZE, TEMP_K
from mlx90640.image import ChessPattern, InterleavedPattern


class MLX_Cam:
    '''!
    @brief      Class which wraps an MLX90640 thermal infrared camera driver to
                make it easier to grab and use an image.
    @details    This image is in "raw" mode, meaning it has not been calibrated
                (which takes lots of time and memory) and only gives relative IR
                emission seen by pixels, not estimates of the temperatures.
    '''

    def __init__(self, i2c, address=0x33, pattern=ChessPattern,
                 width=NUM_COLS, height=NUM_ROWS):
        '''!
        @brief      Set up an MLX90640 camera.
        @param      i2c -> An I2C bus which has been set up to talk to the camera.
                    This must be a bus object which has already been set up.
        @param      address -> The address of the camera on the I2C bus (default 
                    0x33).
        @param      pattern -> The way frames are interleaved, as we read only 
                    half the pixels at a time (default ChessPattern).
        @param      width -> The width of the image in pixels (leave at default).
        @param      height -> The height of the image in pixels (leave at default).
        '''
        ## The I2C bus to which the camera is attached
        self._i2c = i2c
        ## The address of the camera on the I2C bus
        self._addr = address
        ## The pattern for reading the camera, usually ChessPattern
        self._pattern = pattern
        ## The width of the image in pixels, which should be 32
        self._width = width
        ## The height of the image in pixels, which should be 24
        self._height = height
        ## Tracks whether an image is currently being retrieved
        self._getting_image = False
        ## Which subpage (checkerboard half) of the image is being retrieved
        self._subpage = 0

        # The MLX90640 object that does the work
        self._camera = MLX90640(i2c, address)
        self._camera.set_pattern(pattern)
        self._camera.setup()

        ## A local reference to the image object within the camera driver
        self._image = self._camera.raw


    def ascii_image(self, array, pixel="██", textcolor="0;180;0"):
        '''!
        @brief      Show low-resolution camera data as shaded pixels on a text
                    screen.
        @details    The data is printed as a set of characters in columns for 
                    the number of rows in the camera's image size. This function
                    is intended for testing an MLX90640 thermal infrared sensor.

                    A pair of extended ASCII filled rectangles is used by default
                    to show each pixel so that the aspect ratio of the display on 
                    screens isn't too smushed. Each pixel is colored using ANSI
                    terminal escape codes which work in only some programs such as
                    PuTTY. If shown in simpler terminal programs such as the one
                    used in Thonny, the display just shows a bunch of pixel symbols
                    with no difference in shading (boring).

                    A simple auto-brightness scaling is done, setting the lowest 
                    brightness of a filled block to 0 and the highest to 255. If 
                    there are bad pixels, this can reduce contrast in the rest of
                    the image. 

                    After the printing is done, character color is reset to a 
                    default of medium-brightness green, or something else if chosen.
        @param      array -> An array of (self._width * self._height) pixel values.
        @param      pixel -> Text which is shown for each pixel, default being a pair
                    of extended-ASCII blocks (code 219).
        @param      textcolor -> The color to which printed text is reset when the 
                    image has been finished, as a string "<r>;<g>;<b>" with each
                    letter representing the intesity of red, green, and blue from
                    0 to 255.
        '''
        minny = min(array)
        scale = 255.0 / (max(array) - minny)
        for row in range(self._height):
            for col in range(self._width):
                pix = int((array[row * self._width + (self._width - col - 1)]
                           - minny) * scale)
                print(f"\033[38;2;{pix};{pix};{pix}m{pixel}", end='')
            print(f"\033[38;2;{textcolor}m")


    ## A "standard" set of characters of different densities to make ASCII art
    asc = " -.:=+*#%@"


    def ascii_art(self, array):
        '''!
        @brief      Print a data array from the IR image as ASCII art.
        @details    Each character is repeated twice so the image isn't squished
                    latterally. A code of "><" indicates an error, probably caused
                    by a bad pixel in the camera.
        @param      array -> The array to be shown, probably @c image.v_ir.
        @returns    None.
        '''
        scale = len(MLX_Cam.asc) / (max(array) - min(array))
        offset = -min(array)
        for row in range(self._height):
            line = ""
            for col in range(self._width):
                pix = int((array[row * self._width + (self._width - col - 1)]
                           + offset) * scale)
                try:
                    the_char = MLX_Cam.asc[pix]
                    print(f"{the_char}{the_char}", end='')
                except IndexError:
                    print("><", end='')
            print('')
        return


    def get_csv(self, array, limits=None):
        '''!
        @brief      Generate a string containing image data in CSV format.
        @details    This functiton generates a set of lines, each having one row of
                    image data in Comma Separated Variable format. The lines can 
                    be printed or saved to a file useing a @c for loop.
        @param      array -> The array of data to be presented.
        @param      limits -> A 2-iterable containing the maximum and minimum values
                    to which the data should be scaled or @c None for no scaling.
        @returns    None.
        '''
        if limits and len(limits) == 2:
            scale = (limits[1] - limits[0]) / (max(array) - min(array))
            offset = limits[0] - min(array)
        else:
            offset = 0.0
            scale = 1.0
        for row in range(self._height):
            line = ""
            for col in range(self._width):
                pix = int((array[row * self._width + (self._width - col - 1)]
                          + offset) * scale)
                if col:
                    line += ","
                line += f"{pix}"
            yield line
        return
    
    def get_num_csv(self, array, limits=None):
        '''!
        @brief      Generate a list containing image data in CSV format.
        @details    This functiton generates a set of lines, each having one row of
                    image data in Comma Separated Variable format. The lines can 
                    be printed or saved to a file useing a @c for loop.
        @param      array -> The array of data to be presented.
        @param      limits -> A 2-iterable containing the maximum and minimum values
                    to which the data should be scaled or @c None for no scaling.
        @returns    None.
        '''
        if limits and len(limits) == 2:
            scale = (limits[1] - limits[0]) / (max(array) - min(array))
            offset = limits[0] - min(array)
        else:
            offset = 0.0
            scale = 1.0
        for row in range(self._height):
            # line = ""
            line = []
            for col in range(self._width):
                pix = int((array[row * self._width + (self._width - col - 1)]
                          + offset) * scale)
                # if col:
                #     line += ","
                # line += f"{pix}"
                line.append(pix)
            yield line
        return


    def get_image(self):
        '''!
        @brief      Get one image from a MLX90640 camera, @b blocking other tasks
                    from running until the image has been received.
        @details    Grab one image from the given camera and return it. Both 
                    subframes (the odd checkerboard portions of the image) are
                    grabbed and combined (maybe; this is the raw version, so the 
                    combination is sketchy and not fully tested). It is assumed 
                    that the camera is in the ChessPattern (default) mode as it
                    probably should be.
        @param      None.
        @returns    A reference to the image object we've justt filled with data.
        '''
        for subpage in (0, 1):
            while not self._camera.has_data:
                time.sleep_ms(50)
            image = self._camera.read_image(subpage)

        return image


    def get_image_nonblocking(self):
        '''!
        @brief      Get an image from an MLX90640 camera in a non-blocking way.
        @details    This function is to be called repeatedly; it will return @c None
                    until a complete image has been retrieved (this takes around a
                    quarter to half second) and will then return the image.

                @b Example: This code would be inside a task function which yields
                repeatedly as long as there isn't a complete image available.
                @code
                image = None
                while not image:
                    image = camera.get_image_nonblocking()
                    yield(state)
                @endcode

        @param      None.
        @returns    None.
        '''
        # If this is the first recent call, begin the process
        if not self._getting_image:
            self._subpage = 0
            self._getting_image = True
        
        # Read whichever subpage needs to be read, or wait until data is ready
        if not self._camera.has_data:
            return None
        
        image = self._camera.read_image(self._subpage)
        
        # If we just got subpage zero, we need to come back and get subpage 1;
        # if we just got subpage 1, we're done
        if self._subpage == 0:
            self._subpage = 1
            return None
        else:
            self._getting_image = False
            return image


def test_MLX_cam():
    '''!
    This test function sets up the sensor, then grabs and shows an image in a 
    terminal every few seconds. By default it shows ASCII art, but it can be
    set to show better looking grayscale images in some terminal programs such
    as PuTTY. Unfortunately Thonny's terminal won't show the nice grayscale.
    @param      None.
    @returns    None.
    '''
    import gc

    # The following import is only used to check if we have an STM32 board such
    # as a Pyboard or Nucleo; if not, use a different library
    try:
        from pyb import info

    # Oops, it's not an STM32; assume generic machine.I2C for ESP32 and others
    except ImportError:
        # For ESP32 38-pin cheapo board from NodeMCU, KeeYees, etc.
        i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))

    # OK, we do have an STM32, so just use the default pin assignments for I2C1
    else:
        i2c_bus = I2C(1)

    print("MXL90640 Easy(ish) Driver Test")

    # Select MLX90640 camera I2C address, normally 0x33, and check the bus
    i2c_address = 0x33
    scanhex = [f"0x{addr:X}" for addr in i2c_bus.scan()]
    print(f"I2C Scan: {scanhex}")

    # Create the camera object and set it up in default mode
    camera = MLX_Cam(i2c_bus)
    print(f"Current refresh rate: {camera._camera.refresh_rate}")
    camera._camera.refresh_rate = 10.0
    print(f"Refresh rate is now:  {camera._camera.refresh_rate}")

    while True:
        try:
            # Get and image and see how long it takes to grab that image
            print("Click.", end='')
            begintime = time.ticks_ms()
#             image = camera.get_image()

            # Keep trying to get an image; this could be done in a task, with
            # the task yielding repeatedly until an image is available
            image = None
            while not image:
                image = camera.get_image_nonblocking()
                time.sleep_ms(50)

            print(f" {time.ticks_diff(time.ticks_ms(), begintime)} ms")

            # Can show image.v_ir, image.alpha, or image.buf; image.v_ir best?
            # Display pixellated grayscale or numbers in CSV format; the CSV
            # could also be written to a file. Spreadsheets, Matlab(tm), or
            # CPython can read CSV and make a decent false-color heat plot.
            show_image = False
            show_csv = False
            if show_image:
                camera.ascii_image(image)
            elif show_csv:
                i = 0
                for line in camera.get_csv(image, limits=(0, 99)):
                    # l = len(line)
                    # print(f'num row = {i}, num cols = {l}')
                    print(line)
                    i += 1
            else:
                camera.ascii_art(image)
            gc.collect()
            print(f"Memory: {gc.mem_free()} B free")
            time.sleep_ms(500) # time.sleep_ms(3141)

        except KeyboardInterrupt:
            break

    print ("Done.")

# This main code is run if this file is the main program but won't run if this
# file is imported as a module by some other main program
if __name__ == "__main__":

    test_MLX_cam()


