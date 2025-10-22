.. zephyr:code-sample:: ism330bx
   :name: ISM330BX IMU sensor
   :relevant-api: sensor_interface

   Get accelerometer and gyroscope data from an ISM330BX sensor (polling & trigger
   mode).

Overview
********

This sample sets the ISM330BX accelerometer and gyroscope to 60Hz
and enable a trigger on data ready. It displays on the console the
values for accelerometer and gyroscope.


Requirements
************

This sample uses the ISM330BX sensor controlled using the I2C or SPI interface.
This sample uses a special devkit board with a fet powering the ism330bx. And a special pin - ism330bx_en for toggling power.


References
**********

- ISM330BX https://www.st.com/en/mems-and-sensors/ism330bx.html

Building and Running
********************

 This project outputs sensor data to the console. It requires an ISM330BX
 sensor. This project was tested using nrf52840dk and 

Sample Output
=============

.. code-block:: console

    LSM6DSL sensor samples:

    accel (-3.184000 -0.697000 9.207000) m/s2
    gyro (0.065000 -0.029000 0.002000) dps
    - (0) (trig_cnt: 190474)

    <repeats endlessly every 2 seconds>
