# CG1111A
code for NUS mod CG1111A

Hello world

ir detector red
emitter is yellows
l293d is hbridge to on and off the led


## Mbot Specs 
13cm X 19cm 


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

To Do List
Calibrate Ultrasonic sensor done
Calibrate Ir Sensor
Create code for colour sensor
create functions for colour challenges
route correction half


## Color list 
RED {182, 110, 121}
green{60, 121, 85}
blue {97, 166, 145}
orange {182,133, 109}
purple {109, 110 ,121}
White{255,255,223}



if red >145
     if green <160
        red 
    else
        if <150
        orange 
            else
                white

else
    if green>200
        blue  
    
    else if green >100
        green
    else
        purple