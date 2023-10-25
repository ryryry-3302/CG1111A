#include <MeMCore.h>
MeBuzzer buzzer;

#define TURNING_TIME_MS 344.5 // The time duration (ms) for turning

#define TIMEOUT 1000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define ULTRASONIC 10

#define BIT_A_ORANGE A2
#define BIT_B_YELLOW A3
#define LDR A0
#define ir_receiver A1


float colourArray[] = {0,0,0};
float whiteArray[] = {0,0,0};
float blackArray[] = {0,0,0};
float greyDiff[] = {0,0,0};

int red = 0;
int green = 0;
int blue = 0;
int ir_val = 0;

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
uint8_t motorSpeed = 255;

double gen_ultrasonic() {
    pinMode(ULTRASONIC, OUTPUT);
    digitalWrite(ULTRASONIC, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC, LOW);
    pinMode(ULTRASONIC, INPUT);
    long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
    if (duration > 0) {
    double distance = duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100;
   // Serial.print("distance(cm) = ");
   // Serial.println(distance);
    return distance;
    }
    else {
   //  Serial.println("out of range");
    return 0;
    }
}

void celebrate() {
    // Code for playing celebratory tune
    int e = 329;
    int f = 349;
    int g = 370;
    int d = 277;
    int c = 247;
    buzzer.tone(e, 600);
    buzzer.tone(e, 600);
    buzzer.tone(f, 600);
    buzzer.tone(g, 600);
    buzzer.tone(g, 600);
    buzzer.tone(f, 600);
    buzzer.tone(e, 600);
    buzzer.tone(d, 600);
    buzzer.tone(c, 600);
    buzzer.tone(c, 600);
    buzzer.tone(d, 600);
    buzzer.tone(e, 600);
    buzzer.tone(e, 600);
    buzzer.tone(d, 600);
    buzzer.tone(d, 600);
    buzzer.noTone();
    delay(500);
    buzzer.tone(e, 600);
    buzzer.tone(e, 600);
    buzzer.tone(f, 600);
    buzzer.tone(g, 600);
    buzzer.tone(g, 600);
    buzzer.tone(f, 600);
    buzzer.tone(e, 600);
    buzzer.tone(d, 600);
    buzzer.tone(c, 600);
    buzzer.tone(c, 600);
    buzzer.tone(d, 600);
    buzzer.tone(e, 600);
    buzzer.tone(d, 600);
    buzzer.tone(c, 600);
    buzzer.tone(c, 600);
}
void stopMotor() {
    // Code for stopping motor
    leftMotor.stop(); // Stop left motor
    rightMotor.stop();
}
void moveForward() {
    // Code for moving forward for some short interval
    leftMotor.run(motorSpeed); // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed);
}
void turnRight() {
    // Code for turning right 90 deg
    leftMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS); // Keep turning left for this time duration
    leftMotor.stop(); // Stop left motor
    rightMotor.stop();
}
void turnLeft() {
    // Code for turning left 90 deg
    leftMotor.run(motorSpeed); // Positive: wheel turns clockwise
    rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
    delay(TURNING_TIME_MS); // Keep turning left for this time duration
    leftMotor.stop(); // Stop left motor
    rightMotor.stop(); // Stop right motor
    }
void uTurn() {
    // Code for u-turn
    turnLeft();
    delay(1500);
    turnLeft(); 
}
void doubleLeftTurn() {
    // Code for double left turn}
void doubleRightTurn() {
    // Code for double right turn}
void nudgeLeft() {
    // Code for nudging slightly to the left for some short interval 
}
void nudgeRight() {
    // Code for nudging slightly to the right for some short interval 
}
void shineIR() {
    // Code for turning on the IR emitter only
    digitalWrite(BIT_A_ORANGE, HIGH);
    digitalWrite(BIT_B_YELLOW, HIGH);
}
void shineRed() {
    // Code for turning on the red LED only
    digitalWrite(BIT_A_ORANGE, LOW); 
    digitalWrite(BIT_B_YELLOW, LOW); 
}
void shineGreen() {
    // Code for turning on the green LED only
    digitalWrite(BIT_A_ORANGE, LOW); 
    digitalWrite(BIT_B_YELLOW, HIGH);
}
void shineBlue() {
    // Code for turning on the blue LED only
    digitalWrite(BIT_A_ORANGE, HIGH); 
    digitalWrite(BIT_B_YELLOW, LOW);
}

int detectColour()
{
// Shine Red, read LDR after some delay
// Shine Green, read LDR after some delay
// Shine Blue, read LDR after some delay
// Run algorithm for colour decoding
}
void setup()
{
// Configure pinMode for A0, A1, A2, A3
Serial.begin(9600); // to initialize the serial monitor
pinMode(BIT_A_ORANGE, OUTPUT);
pinMode(BIT_B_YELLOW, OUTPUT);
pinMode(LDR, INPUT);

}
void loop()
{
// Read ultrasonic sensing distance (choose an appropriate timeout)
int right_distance = gen_ultrasonic();
delay(500);

// Read IR sensing distance (turn off IR, read IR detector, turn on IR, read IR detector, estimate distance)
// if within black line, stop motor, detect colour, and take corresponding action
// else if too near to left wall, nudge right
// else if too near to right wall, nudge left
// else move forward
}
