'''!
@file test.py

A file to help test other files and other code

'''

# import pyb 
from machine import I2C
import utime
import mlx_cam


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
        image = camera.get_image_nonblocking() # doing this once takes about 157ms
        print(f'time to check image = {utime.ticks_diff(utime.ticks_ms(),start_time)} ms')
    # indicate done with this test
    print('Done testing image nonblock time.')

def main():
    '''!
    This function only runs when this file is ran as the main file.
    @param      None.
    @returns    None.
    '''
    test_camera_nonblock()

if __name__ == '__main__':
    main()
