# ME 405 Term Project

## Authors
Jack Krammer and Jason Chang

Assisting professors: 
* Ridgely, John R.
* Refvem, Charlie Thomas

California Polytechnic State University

March 18, 2024

## Description
For the Winter Quarter term of ME 405's lab, we were assigned by Dr. John Ridgely 
to create a **heat-sensing foam dart blaster turret**. Its capabilites include full 
autonomous motion, aiming, and firing. For aiming it would use a lab provided 
thermal infrared camera (model MLX90640) to find its intended target.

Our device was tested against others on Dueling Day where along the ends of a long 
table each opposing device was secured and a single member of each team stood behind 
their respective device. Upon the "duel" beginning, members would move within a 5 
second period, where afterwards they must remain frozen in their chosen position and 
each oppposing device must aim and fire against their opponent within 10 seconds 
afterwards. For further clarification upon project and Dueling Day rules, you may go 
to Dr. John Ridgely's provided HTML file below.
<a href="./src/links/termproj_W24.html" title="term_proj_specs">Term Project Specifications</a>


## Dependencies
This project depends on the MicroPython 
<a href="https://docs.micropython.org/en/latest/library/pyb.html" title="pyb">pyb</a>,
<a href="https://docs.micropython.org/en/latest/library/machine.I2C.html" title="machine I2C">machine.I2C</a>,
<a href="https://docs.micropython.org/en/v1.15/library/utime.html" title="uitme">utime</a>,
<a href="https://docs.python.org/3/library/math.html" title="math">math</a>,
and 
<a href="https://docs.python.org/3/library/gc.html" title="gc">gc</a>
libraries as well as 
<a href="https://github.com/spluttflob/ME405-Support/tree/main/mlx_raw" title="mlx_cam">mlx_cam</a>
and 
<a href="https://github.com/spluttflob/ME405-Support/blob/main/src/cotask.py" title="cotask">cotask</a>
with some associated documentation found 
<a href="https://spluttflob.github.io/ME405-Support/" title="ME 405 documentatin">here</a>.





