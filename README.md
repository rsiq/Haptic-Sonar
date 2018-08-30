HAPTIC SONAR PROJECT
------------------------------------------------------------------------------------------------
				

Welcome to my project!

OVERVIEW:
------------------------------------------------------------------------------------------------

Disclaimer: This project is intended for educational purposes only, it is not intended for use
	    in an unsupervised environment.

The objective of this project was to create an environmental detection system for people
with severe visual impairment. The approach was to use ultrasonic sensors to calculate the depth
of objects within their field of view. The result of that was then converted into a PWM pulse-
width via a lookup table, and outputted to small DC vibration motors to provide haptic feedback
to the user. 

The system is intended to have four sonar sensors mounted on the user's head with a hat
pointing in the x, -x, y, and -y directions. The four vibrational motors are intended to be
placed anywhere on the user's body that is safe, comfortable, and provides good contact for
haptic feedback. The ultrasonic sensors and vibrational motors are intended to be connected to 
the microcontroller via GPIO and PWM respectively.


TECHNICAL SPECIFICATIONS:
------------------------------------------------------------------------------------------------

-Compiled with: Keil uVision v5

-Target Device: Tiva TM4C123G Microcontroller

-Ultrasonic Sensor: HC-SR04

-Vibrational Motor: Adafruit 1201 Vibe Motor

