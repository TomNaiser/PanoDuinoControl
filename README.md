# PanoDuinoControl
Arduino  based control software for an automated pan/tilt platform for Gigapixel Panorama Acquisition

Features:

-wireless communication with PanoramaControl front end

-PID closed loop position control of  modified RC-Servos (operated via external H-Bridge)

-high positional accuracy by using 14-bit rotary encoder data

-remote triggering of camera shutters 


Requirements:  

-PanoramaControl software (which can also be found in my Github repository)

-Arduino Lib for AMS AS5048B I2C - 14-bit magnetic rotary position sensor 
https://github.com/sosandroid/AMS_AS5048B   (under the BSD license)
