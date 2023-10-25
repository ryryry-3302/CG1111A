# CG1111A
code for NUS mod CG1111A

Hello world

ir detector red
emitter is yellows
l293d is hbridge to on and off the led

## Port 1 Black line detector

## Port 2 Ultrasonic
Ultrasonic pin D10

## Port3 2D to 4D
red vcc
black ground
yellow s2 A3
orange s1 A2


## M4 data readers
grey ground
white vcc 
purple s1  A0 LDR
blue s2    A1  ir receveier


## Motor config
Left Motor = M1
Right Motor = M2
leftMotor.run(200); // Left wheel goes forward (anti-clockwise)
rightMotor.run(-200);

Colour Interpretation
Red Left-turn
Green Right turn
Orange 180° turn within the same grid
Purple Two successive left-turns in two grids
Light Blue Two successive right-turns in two grids

test