# Nova SDS011 Particle Sensor for Arduino / any Mcu
 Simple and lean SDS011 Uart interfacing<BR>
 No Library-overhead, just one straight forward code example to serial.<BR>
 
 # Project Example for PM10 / PM2.5
 See project Directory for an example. Used to build a real sensor, and have it been part of the Sensor Community Network<BR>
+ [HacksterIO project](https://www.hackster.io/voske65/particle-sensor-nova-sds011-nano33iot-073b3b)<BR><BR>
+ ![Board](/project/PM10/images/Sensor_setup%20jpg.jpg?raw=true)<BR>


 # Pinning
You'll need to connect the RX and TX pins of the sensor to the corresponding TX and RX pins (yes, swop them) on your microcontroller.<BR>
Connect 5V(!) and Gnd to power lines, see example of SDS011 sensor datasheet<BR>
I2C Bus Address is 0x69<BR>
<br><br>
+ ![Board](/images/SDS011.jpg?raw=true)
<br><br>
More info :<BR>

+ [Nova SDS011 Datasheet](https://hollandse-luchten.org/wp-content/uploads/sites/9/nova_laser_sensorSDS011datasheet.pdf)
+ [Tiny Tronics](https://www.tinytronics.nl/en/sensors/air/dust/nova-sds011-high-precision-laser-dust-sensor)

