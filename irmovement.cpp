#include <MeMCore.h>
#include <PID_v1.h>
MeBuzzer buzzer;
MeLineFollower lineFinder(PORT_1);

#define TURNING_TIME_MS 344.5 // The time duration (ms) for turning

#define TIMEOUT 1100 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define ULTRASONIC 10 // Ultrasonic Sensor Pin

#define BIT_A_ORANGE A2 // S1
#define BIT_B_YELLOW A3 // S2

#define LDR A0 //Pin of LDR
#define ir_receiver A1 // Pin of IR Receiver


unsigned long current_time = 0;
//Define Variables we'll be connecting to
double ambient, LeftInput, Output, ambient_without_ir;

//Specify the links and initial tuning parameters
double Kp=1, Ki=0.2, Kd=0;

PID leftPID(&LeftInput, &Output, &ambient, Kp, Ki, Kd, DIRECT);


int right_distance = 1; //distance from right wall
int left_distance = 0; //distance from left wall

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
uint8_t motorSpeed = 255;
uint8_t lowSpeed  = motorSpeed - 150;



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
    double distance = duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100 - 3.40;
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
    delay(1500);
    leftMotor.stop(); // Stop left motor
    rightMotor.stop(); 
}
void doubleLeftTurn() {
    }
void doubleRightTurn() {
    }
void nudgeRight() {
    leftMotor.run(lowSpeed); 
    rightMotor.run(-motorSpeed);
    }
void nudgeLeft() {

    leftMotor.run(motorSpeed); 
    rightMotor.run(-lowSpeed); 
 

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

double ambient_ir(){
  double voltage = 0;
  shineRed();
  if(millis() > current_time + 30)
  { 
    ambient_without_ir = analogRead(ir_receiver);
    shineIR();
  }
  
  if (millis() > current_time + 60){
    current_time=millis();
    double temp = 0;
    for (int i=0; i<5; i++){
    temp += analogRead(ir_receiver);
    voltage = ambient_without_ir - temp / 5;
  }
  
  }
   
  return voltage;
}



void setup()
{
// Configure pinMode for A0, A1, A2, A3
Serial.begin(9600); // to initialize the serial monitor
pinMode(BIT_A_ORANGE, OUTPUT);
pinMode(BIT_B_YELLOW, OUTPUT);
pinMode(LDR, INPUT);
delay(100);
ambient = ambient_ir();
Serial.print("ambient");
Serial.println(ambient);
leftPID.SetMode(AUTOMATIC);
}
void loop()
{
    
        LeftInput = ambient_ir();
        if( LeftInput != 0){
        Serial.println();
        }

        if ((ambient - LeftInput < 35) && LeftInput != 0){ 

        if(ambient - LeftInput < 25){
          nudgeRight();
          delay(20);
        }
        else if (ambient - LeftInput > 30){
          nudgeLeft();
          delay(20);

        }
       
        else{
          leftMotor.run(lowSpeed); // Positive: wheel turns clockwise
          rightMotor.run(-lowSpeed);
        }
        }
        else{
          leftMotor.run(lowSpeed); // Positive: wheel turns clockwise
          rightMotor.run(-lowSpeed);
        }

        


   
    
    


// Read IR sensing distance (turn off IR, read IR detector, turn on IR, read IR detector, estimate distance)
// if within black line, stop motor, detect colour, and take corresponding action
// else if too near to left wall, nudge right
// else if too near to right wall, nudge left
// else move forward
}