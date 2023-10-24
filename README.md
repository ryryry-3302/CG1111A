# CG1111A
code for NUS mod CG1111A

Hello world

ir detector red
emitter is yellows
l293d is hbridge to on and off the led

m1
black line detector

m2
ultrasonic d9
          d10

m3
red vcc
black ground
yellow s2 
orange s1


m4
grey ground
white vcc 
purple s1
blue s2 ir  receveier


motor config
M1 leftMotor.run(200); // Left wheel goes forward (anti-clockwise)
M2 rightMotor.run(-200);

Colour Interpretation
Red Left-turn
Green Right turn
Orange 180° turn within the same grid
Purple Two successive left-turns in two grids
Light Blue Two successive right-turns in two grids
